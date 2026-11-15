% The tactic's plan, in MiniZinc. A MiniZinc run takes tens of
% milliseconds, too slow to repeat every 50 ms turn, so the SDK solves this
% model once, at the hello, with that hello's numbers (clunkyard.mzn
% declares them), and bot.py reads the plan each turn. For every
% whole-degree error from -180 to 180 it gives the turn to command; for
% every 10 px of range, the shot power and the scan arc that reaches a cue;
% the bands below choose the speed and when to fire.
%
% Bearing is earned: a scan contact's bearing is off by up to its
% bearing_error (degrees, a fixed share of the arc), so a wide beam finds a
% robot and a narrow one aims at it. The plan also gives, for every 10 px of
% range, the narrowest beam that still holds the target in it, and the
% constants of the line bot.py fits through the recent sightings.
include "clunkyard.mzn";

array[-180..180] of float: steer = core_steer;
% Sharper steering away from the storm and back inside the zone.
array[-180..180] of float: home = core_home;

% The heaviest shot that still covers each range in about a second, and the
% widest scan that reaches 100 px past it.
int: range_step = core_range_step;
int: steps = core_range_steps;
array[0..steps] of float: power = core_power;
array[0..steps] of float: cue_arc = core_cue_arc;

% The beam that is hold_width across at each range: narrow enough for a
% precise bearing, wide enough to keep the target in it from tick to tick.
float: hold_width = 60.0;           % units across at the target
float: min_arc = 6.0;               % degrees: the narrowest the beam holds
array[0..steps] of float: hold_arc = array1d(0..steps,
  [ if r == 0 then 180.0 else max(min_arc, 2.0 * atan(hold_width / 2.0 / int2float(r * range_step)) * 180.0 / 3.141592653589793) endif | r in 0..steps ]);
int: window = 12;                   % ticks of sightings the aim averages
int: sightings = 8;                 % narrow sightings in the window before a shot
int: look_every = 40;               % ticks between looks around the target, for a nearer robot
int: reacquire = 10;                % ticks to look again where a lost target was

float: near_range = 200.0;          % back off inside this
float: far_range = 350.0;           % close in beyond this
float: track_arc = 20.0;            % the fixed beam of a robot that does not earn its bearing
float: search_arc = 40.0;           % sweeps the circle in half a second
float: search_turn = 0.33;          % enough to turn against the brake in the force rulesets
float: aim_tolerance = 3.0;         % degrees between the turret and the shot
float: reserve = 10.0;              % battery kept back after a shot
float: low_battery = battery_capacity / 4;
float: keep_out = 60.0;             % distance kept from the storm's edge and the zone's
% A simple dodge when a shot is heard, off the line of fire for half a
% second: the throttle and turn with direct control, the throttle and
% strafe with forces.
int: dodge_ticks = 10;
float: dodge_throttle = 1.0;
float: dodge_turn = 1.0;
float: dodge_force_throttle = 0.5;
float: dodge_strafe = 1.0;

output ["{\"steer\":", showJSON(steer), ",\"home\":", showJSON(home),
        ",\"hold_arc\":", showJSON(hold_arc), ",\"min_arc\":", showJSON(min_arc), ",\"window\":", showJSON(window),
        ",\"sightings\":", showJSON(sightings), ",\"look_every\":", showJSON(look_every), ",\"reacquire\":", showJSON(reacquire),
        ",\"range_step\":", showJSON(range_step), ",\"power\":", showJSON(power), ",\"cue_arc\":", showJSON(cue_arc),
        ",\"near_range\":", showJSON(near_range), ",\"far_range\":", showJSON(far_range),
        ",\"track_arc\":", showJSON(track_arc), ",\"search_arc\":", showJSON(search_arc),
        ",\"search_turn\":", showJSON(search_turn), ",\"aim_tolerance\":", showJSON(aim_tolerance),
        ",\"reserve\":", showJSON(reserve), ",\"low_battery\":", showJSON(low_battery),
        ",\"keep_out\":", showJSON(keep_out), ",\"dodge_ticks\":", showJSON(dodge_ticks),
        ",\"dodge_throttle\":", showJSON(dodge_throttle), ",\"dodge_turn\":", showJSON(dodge_turn),
        ",\"dodge_force_throttle\":", showJSON(dodge_force_throttle), ",\"dodge_strafe\":", showJSON(dodge_strafe), "}\n"];
