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
# Clunkyard robot API 2.1.0: one core under every language, the same
# answers everywhere. Facts (a call shows its arguments):
# match: mode ruleset control robots is_3d tick_rate max_ticks
#   robot_id(slot)
# clock: tick seconds remaining_ticks
# arena: size gravity terrain_kind ground_height(x y)
# me: health position velocity heading pitch roll angular_velocity
#   attitude grounded altitude wall_contact hurt applied hull_mounts
#   to_mount(heading pitch) from_mount(heading pitch) mount_toward(point)
# battery: charge capacity regen sustainable_drive compute
#   drive_cost(fraction)
# drive: hover_lift
# gun: aim_now cooldown_ticks spread max_power damage_scale
#   can_fire(power reserve) shot_speed(power) shot_cost(power)
#   reload_ticks(power) shot_damage(power) power_for_speed(speed)
#   power_for_flight_time(range seconds) lead(contact power)
#   lead_point(position velocity power)
# scanner: last proximity_range reach(arc) arc_for_reach(range) is_on
# sensors: contacts nearest cues contact(slot) position_of(contact)
# tracker: cue nearest_bearing_rate velocity(slot) position(slot)
#   history(slot) last_seen(slot)
# nav: zone zone.margin(point) zone.contains(point margin)
#   zone.distance_from(point) storm storm.margin(point)
#   storm.contains(point margin) storm.distance_from(point) hill
#   hill.margin(point) hill.contains(point margin)
#   hill.distance_from(point) wall_margin() pickups nearest_pickup(kind)
#   field_distance(a b)
# score: me best armour
# drone: has ready in_flight launches_left report sightings
# angles: normalize(degrees) angle_diff(target current)
#   bearing_to(from to) elevation_to(from to) direction(heading pitch)
#   distance(a b) to_chassis(heading pitch attitude)
#   from_chassis(heading pitch attitude) clamp(value low high)
# raw: number(path) text(path) has(path) flag(path) count(path) line()
# Controls: drive.set(throttle turn pitch_turn)
#   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
#   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
#   scanner.power(on) tracker.configure(smoothing forget_ticks cue_ticks)
#   drone.launch(heading pitch param) drone.brief(index value)
#   drone.program(language source) status.set(text value) debug.log(line)
#   raw.extra(json)
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
        self.dodges, self.dodge_end = 0, 0  # dodges taken, and the tick the current one ends
        self.slot = 0  # the last target's slot, for looking again where it was lost

    def fit(self, slot, tick):
        """A straight line through the narrowest of the last plan.window ticks'
        sightings: where the robot is now and how fast it moves. Each sighting
        is off by its bearing_error; the line averages that out."""
        recent = [s for s in self.tracker.history(slot) if s.tick > tick - self.plan.window]
        if len(recent) < 2:
            return None
        best = min(s.bearing_error for s in recent)
        seen = [s for s in recent if s.bearing_error <= 1.5 * best]  # the narrow ones
        n = len(seen)
        if n < 2:
            return None
        t0 = sum(s.tick for s in seen) / n
        spread = sum((s.tick - t0) ** 2 for s in seen)
        position, velocity = [], []
        for axis in "xyz":
            mean = sum(getattr(s.position, axis) for s in seen) / n
            slope = sum((s.tick - t0) * (getattr(s.position, axis) - mean) for s in seen) / spread
            position.append(mean + slope * (tick - t0))
            velocity.append(slope * self.match.tick_rate)
        return cy.Vec3(*position), cy.Vec3(*velocity), n

    def steer(self, off, table="steer"):
        """The planned turn for an angle off, full at 45 degrees (30 for home)."""
        return self.plan.at(table, angles.angle_diff(off, 0), first=-180)

    def on_turn(self):
        me, gun, scanner, plan = self.me, self.gun, self.scanner, self.plan
        self.drive.forces(brake=0.2)
        target = self.sensors.nearest
        cue = self.tracker.cue  # the freshest passive contact, for a few seconds
        tick = self.clock.tick
        last_seen = self.tracker.last_seen(self.slot) if self.slot else None
        if target:
            self.slot = target.slot
            throttle = 0.7 if target.range > plan.far_range else -0.4 if target.range < plan.near_range else 0.2
            self.drive.set(throttle=throttle, turn=self.steer(target.bearing - me.heading),
                           pitch_turn=self.steer(target.elevation - me.pitch))
            # Halve the beam each tick until it is as narrow as the plan says
            # for this range: narrow enough for a precise bearing, wide
            # enough to keep it in. In a crowd a narrow beam sees one robot, so
            # every look_every ticks widen it for a tick: a nearer robot it
            # finds becomes the target.
            hold = plan.at("hold_arc", target.range, step=plan.range_step)
            arc = plan.search_arc if tick % plan.look_every == 0 else max(hold, min(plan.search_arc, scanner.last.arc / 2))
            scanner.set(heading=target.bearing, arc=arc, pitch=target.elevation)
            # The planned power for this range, aimed where the target will
            # be, lobbed under gravity.
            power = plan.at("power", target.range, step=plan.range_step)
            # An exact contact: gun.lead is the intercept if the target holds
            # course. A noisy one: lead the fitted line, and fire only once it
            # rests on enough sightings.
            ready = True
            if target.bearing_error > 0:
                line = self.fit(target.slot, tick)
                aim = gun.lead_point(line[0], line[1], power) if line else None
                ready = line is not None and line[2] >= plan.sightings
            else:
                aim = gun.lead(target, power)
            heading, pitch = (aim.heading, aim.pitch) if aim else (target.bearing, target.elevation)
            gun.aim(heading, pitch)
            now, tolerance = gun.aim_now, plan.aim_tolerance
            on_target = abs(angles.angle_diff(heading, now.heading)) < tolerance and abs(pitch - now.pitch) < tolerance
            if ready and on_target and gun.can_fire(power, plan.reserve):
                gun.fire(power)
            self.status.set("TRACK" if ready else "NARROW", target.range)
        elif last_seen is not None and tick - last_seen <= plan.reacquire:
            # Lost it: look again where it was last seen, widening the beam, before searching.
            where = self.tracker.position(self.slot) or me.position
            bearing = angles.bearing_to(me.position, where)
            elevation = angles.elevation_to(me.position, where) if self.free else 0
            self.drive.set(throttle=0.5, turn=self.steer(bearing - me.heading), pitch_turn=self.steer(elevation - me.pitch))
            scanner.set(heading=bearing, arc=min(plan.search_arc, scanner.last.arc * 2), pitch=elevation)
            self.status.set("REACQUIRE")
        elif cue:
            # Head for where it was heard, sweeping an arc that still reaches
            # it across the bearing's error (and, flying free, the elevation's).
            bearing, distance = angles.bearing_to(me.position, cue.position), angles.distance(me.position, cue.position)
            elevation = angles.elevation_to(me.position, cue.position) if self.free else 0
            error = min(cue.bearing_error, 30)
            across = (tick % 5 - 2) / 2 * error
            up = ((tick // 5) % 3 - 1) * error / 2 if self.free else 0
            self.drive.set(throttle=0.7, turn=self.steer(bearing - me.heading), pitch_turn=self.steer(elevation - me.pitch))
            scanner.set(heading=bearing + across, arc=plan.at("cue_arc", distance, step=plan.range_step), pitch=elevation + up)
            self.status.set("CUE", distance)
        else:
            self.drive.set(throttle=0.5, turn=plan.search_turn, pitch_turn=self.steer(-me.pitch))
            # Sweeps the circle in half a second, flying free at three heights in turn.
            pitch = ((tick // 10) % 3 - 1) * 35 if self.free else 0
            scanner.set(heading=(scanner.last.heading + 36) % 360, arc=plan.search_arc, pitch=pitch)
            self.status.set("SEARCH")
        # A simple dodge: vary it, or predict the shooter.
        if tick >= self.dodge_end and any(cue.source == "muzzle" for cue in self.sensors.cues):
            self.dodges += 1
            self.dodge_end = tick + plan.dodge_ticks
        if tick < self.dodge_end:
            # Off the line of fire: left, then right next time (turn is positive left, strafe positive right).
            left = self.dodges % 2 == 1
            if self.match.control == "forces":
                self.drive.set(throttle=plan.dodge_force_throttle)
                self.drive.forces(strafe=-plan.dodge_strafe if left else plan.dodge_strafe)
            else:
                self.drive.set(throttle=plan.dodge_throttle, turn=plan.dodge_turn if left else -plan.dodge_turn)
            self.status.set("dodge:left" if left else "dodge:right", self.dodges)
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


if __name__ == "__main__":
    cy.run(Starter(), binding_name="clunkyard-minizinc")
