# A starter robot for rules v2, in MiniZinc, on the Clunkyard Python binding
# (the clunkyard package, which the arena provides) and its installed MiniZinc adapter. It sweeps a
# medium scan to find a robot, or points it where a passive contact was
# heard, narrows the scan to hold it, leads it with shots whose power suits
# the range, fires only when the turret is on target, keeps battery back for
# driving and keeps out of the storm. tactic.mzn plans the numbers once, at
# the hello; each turn this file plays from that plan (self.plan).
#
# The trade-offs (self.gun, self.scanner and self.battery, and the model's
# parameters, have the numbers):
#   scan     a narrow arc reaches further; a wide one finds robots nearby
#   power    heavy shots hit harder for less battery, but fly slowly
#   battery  shots and driving share it, and it refills slowly
#   turret   it turns at a limited rate; self.gun.aim_now is where it points now
#   cues     passive contacts (a ping, a scan beam, a shot heard) say which
#            way to look, not where to shoot: they have no id and a noisy
#            bearing, so self.sensors.nearest leaves them out
# Clunkyard robot API 2.0.0: one core under every language, the same
# answers everywhere. Facts (a call shows its arguments):
# match: mode ruleset control robots is_3d tick_rate max_ticks
#   robot_id(slot)
# clock: tick seconds remaining_ticks
# arena: size gravity terrain_kind ground_height(x y)
# me: health position velocity heading pitch roll angular_velocity
#   attitude grounded altitude applied
# battery: charge capacity regen sustainable_drive compute
#   drive_cost(fraction)
# drive: hover_lift
# gun: aim_now cooldown_ticks spread max_power damage_scale
#   can_fire(power reserve) shot_speed(power) shot_cost(power)
#   reload_ticks(power) shot_damage(power) power_for_speed(speed)
#   power_for_flight_time(range seconds) lead(contact power)
#   lead_point(position velocity power)
# scanner: last proximity_range reach(arc) arc_for_reach(range)
# sensors: contacts nearest cues contact(slot) position_of(contact)
# tracker: cue nearest_bearing_rate velocity(slot) position(slot)
#   last_seen(slot)
# nav: zone zone.margin(point) zone.contains(point margin)
#   zone.distance_from(point) storm storm.margin(point)
#   storm.contains(point margin) storm.distance_from(point) hill
#   hill.margin(point) hill.contains(point margin)
#   hill.distance_from(point) pickups nearest_pickup(kind)
#   field_distance(a b)
# score: me best armour
# angles: normalize(degrees) angle_diff(target current)
#   bearing_to(from to) elevation_to(from to) direction(heading pitch)
#   distance(a b) clamp(value low high)
# raw: number(path) text(path) has(path) flag(path) count(path) line()
# Controls: drive.set(throttle turn pitch_turn)
#   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
#   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
#   tracker.configure(smoothing forget_ticks cue_ticks)
#   status.set(text value) raw.extra(json)
# Every control argument is optional; a second call keeps what the first
# set. Absent facts are your language's none. Names are snake_case here;
# your language cases them its own way. Angles are degrees from the +x
# axis; pitch is positive up.
# Manual: https://clunkyard.com/manual#sdk
import math

import clunkyard as cy
import clunkyard_minizinc as mz

angles = cy.angles


class Starter(mz.Planner):
    model = "tactic.mzn"

    def on_start(self):
        super().on_start()  # solves the model
        self.free = self.match.is_3d and self.arena.gravity.z == 0  # flying free, robots may be above or below

    def steer(self, off, table="steer"):
        """The planned turn for an angle off, full at 45 degrees (30 for home)."""
        return self.plan.at(table, angles.angle_diff(off, 0), first=-180)

    def on_turn(self):
        me, gun, scanner, plan = self.me, self.gun, self.scanner, self.plan
        self.drive.forces(brake=0.2)
        target = self.sensors.nearest
        cue = self.tracker.cue  # the freshest passive contact, for a few seconds
        if target:
            throttle = 0.7 if target.range > plan.far_range else -0.4 if target.range < plan.near_range else 0.2
            self.drive.set(throttle=throttle, turn=self.steer(target.bearing - me.heading),
                           pitch_turn=self.steer(target.elevation - me.pitch))
            scanner.set(heading=target.bearing, arc=plan.track_arc, pitch=target.elevation)
            # The planned power for this range, aimed where the target will
            # be, lobbed under gravity.
            power = plan.at("power", target.range, step=plan.range_step)
            aim = gun.lead(target, power)
            heading, pitch = (aim.heading, aim.pitch) if aim else (target.bearing, target.elevation)
            gun.aim(heading, pitch)
            now, tolerance = gun.aim_now, plan.aim_tolerance
            on_target = abs(angles.angle_diff(heading, now.heading)) < tolerance and abs(pitch - now.pitch) < tolerance
            if on_target and gun.can_fire(power, plan.reserve):
                gun.fire(power)
            self.status.set("TRACK", target.range)
        elif cue:
            # Head for where it was heard, sweeping an arc that still reaches
            # it across the bearing's error (and, flying free, the elevation's).
            bearing, distance = angles.bearing_to(me.position, cue.position), angles.distance(me.position, cue.position)
            elevation = angles.elevation_to(me.position, cue.position) if self.free else 0
            error = min(cue.bearing_error, 30)
            tick = self.clock.tick
            across = (tick % 5 - 2) / 2 * error
            up = ((tick // 5) % 3 - 1) * error / 2 if self.free else 0
            self.drive.set(throttle=0.7, turn=self.steer(bearing - me.heading), pitch_turn=self.steer(elevation - me.pitch))
            scanner.set(heading=bearing + across, arc=plan.at("cue_arc", distance, step=plan.range_step), pitch=elevation + up)
            self.status.set("CUE", distance)
        else:
            self.drive.set(throttle=0.5, turn=plan.search_turn, pitch_turn=self.steer(-me.pitch))
            # Sweeps the circle in half a second, flying free at three heights in turn.
            pitch = ((self.clock.tick // 10) % 3 - 1) * 35 if self.free else 0
            scanner.set(heading=(scanner.last.heading + 36) % 360, arc=plan.search_arc, pitch=pitch)
            self.status.set("SEARCH")
        # Low battery: drive gently so the shots can still be paid for.
        if self.battery.charge < plan.low_battery:
            self.drive.set(throttle=self.drive.get("throttle") * 0.5)
        # The storm hurts inside: drive straight away from its centre.
        storm = self.nav.storm
        if storm and storm.contains(me.position, plan.keep_out):
            self.drive.set(throttle=0.8, turn=self.steer(angles.bearing_to(storm.center, me.position) - me.heading, "home"))
        # Sudden death: head back inside the safe zone.
        zone = self.nav.zone
        if zone and zone.margin(me.position) < plan.keep_out:
            climb = angles.elevation_to(me.position, zone.center) if zone.shape == "sphere" else 0
            self.drive.set(throttle=0.8, turn=self.steer(angles.bearing_to(me.position, zone.center) - me.heading, "home"),
                           pitch_turn=self.steer(climb - me.pitch, "home"))
        calibrate(self)


# ---- The arena's rate calibration: safe to delete, with the line above ----
# Only the calibration sends calibration.work; it times this language on the
# same work as every other. The kernel is xorshift-sqrt v1: work rounds of a
# 64-bit xorshift and a square-root sum, reported as WORK.
def calibrate(robot):
    work = robot.raw.number("state.calibration.work")
    if not work or work < 1:
        return
    mask, x, total = 0xFFFFFFFFFFFFFFFF, 88172645463325252 ^ robot.clock.tick, 0.0
    for _ in range(int(work)):
        x ^= (x << 13) & mask
        x ^= x >> 7
        x ^= (x << 17) & mask
        total += math.sqrt(x >> 11)
    robot.status.set("WORK", (math.floor(total) + (x >> 40)) % 1000000)


if __name__ == "__main__":
    cy.run(Starter(), binding_name="clunkyard-minizinc")
