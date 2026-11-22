# Play each turn using the plan solved in tactic.mzn at match start.
# Clunkyard guide: https://clunkyard.com/sdk/minizinc
import math

import clunkyard as cy
import clunkyard_minizinc as mz

angles = cy.angles


class Starter(mz.Planner):
    model = "tactic.mzn"

    def on_start(self):
        super().on_start()  # solves the model
        self.free = (
            self.match.is_3d and self.arena.gravity.z == 0
        )  # flying free, robots may be above or below
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
            throttle = (
                0.7
                if target.range > plan.far_range
                else -0.4
                if target.range < plan.near_range
                else 0.2
            )
            self.drive.set(
                throttle=throttle,
                turn=self.steer(target.bearing - me.heading),
                pitch_turn=self.steer(target.elevation - me.pitch),
            )
            # Halve the beam each tick until it is as narrow as the plan says
            # for this range: narrow enough for a precise bearing, wide
            # enough to keep it in. In a crowd a narrow beam sees one robot, so
            # every look_every ticks widen it for a tick: a nearer robot it
            # finds becomes the target.
            hold = plan.at("hold_arc", target.range, step=plan.range_step)
            arc = (
                plan.search_arc
                if tick % plan.look_every == 0
                else max(hold, min(plan.search_arc, scanner.last.arc / 2))
            )
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
            on_target = (
                abs(angles.angle_diff(heading, now.heading)) < tolerance
                and abs(pitch - now.pitch) < tolerance
            )
            if ready and on_target and gun.can_fire(power, plan.reserve):
                gun.fire(power)
            self.status.set("TRACK" if ready else "NARROW", target.range)
        elif last_seen is not None and tick - last_seen <= plan.reacquire:
            # Lost it: look again where it was last seen, widening the beam, before searching.
            where = self.tracker.position(self.slot) or me.position
            bearing = angles.bearing_to(me.position, where)
            elevation = angles.elevation_to(me.position, where) if self.free else 0
            self.drive.set(
                throttle=0.5,
                turn=self.steer(bearing - me.heading),
                pitch_turn=self.steer(elevation - me.pitch),
            )
            scanner.set(
                heading=bearing, arc=min(plan.search_arc, scanner.last.arc * 2), pitch=elevation
            )
            self.status.set("REACQUIRE")
        elif cue:
            # Head for where it was heard, sweeping an arc that still reaches
            # it across the bearing's error (and, flying free, the elevation's).
            bearing, distance = (
                angles.bearing_to(me.position, cue.position),
                angles.distance(me.position, cue.position),
            )
            elevation = angles.elevation_to(me.position, cue.position) if self.free else 0
            error = min(cue.bearing_error, 30)
            across = (tick % 5 - 2) / 2 * error
            up = ((tick // 5) % 3 - 1) * error / 2 if self.free else 0
            self.drive.set(
                throttle=0.7,
                turn=self.steer(bearing - me.heading),
                pitch_turn=self.steer(elevation - me.pitch),
            )
            scanner.set(
                heading=bearing + across,
                arc=plan.at("cue_arc", distance, step=plan.range_step),
                pitch=elevation + up,
            )
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
                self.drive.set(
                    throttle=plan.dodge_throttle, turn=plan.dodge_turn if left else -plan.dodge_turn
                )
            self.status.set("dodge:left" if left else "dodge:right", self.dodges)
        # Low battery: drive gently so the shots can still be paid for.
        if self.battery.charge < plan.low_battery:
            self.drive.set(throttle=self.drive.get("throttle") * 0.5)
        # The storm hurts inside: drive straight away from its centre.
        storm = self.nav.storm
        if storm and storm.contains(me.position, plan.keep_out):
            self.drive.set(
                throttle=0.8,
                turn=self.steer(angles.bearing_to(storm.center, me.position) - me.heading, "home"),
            )
        # Sudden death: head back inside the safe zone.
        zone = self.nav.zone
        if zone and zone.margin(me.position) < plan.keep_out:
            climb = angles.elevation_to(me.position, zone.center) if zone.shape == "sphere" else 0
            self.drive.set(
                throttle=0.8,
                turn=self.steer(angles.bearing_to(me.position, zone.center) - me.heading, "home"),
                pitch_turn=self.steer(climb - me.pitch, "home"),
            )


if __name__ == "__main__":
    mz.run(Starter())
