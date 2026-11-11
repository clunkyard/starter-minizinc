% The tactic's plan, in MiniZinc. A MiniZinc run takes tens of
% milliseconds, too slow to repeat every 50 ms turn, so the SDK solves this
% model once, at the hello, with that hello's numbers (clunkyard.mzn
% declares them), and bot.py reads the plan each turn. For every
% whole-degree error from -180 to 180 it gives the turn to command; for
% every 10 px of range, the shot power and the scan arc that reaches a cue;
% the bands below choose the speed and when to fire.
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

float: near_range = 200.0;          % back off inside this
float: far_range = 350.0;           % close in beyond this
float: track_arc = 20.0;            % narrow: holds it, reaching about 735
float: search_arc = 40.0;           % sweeps the circle in half a second
float: search_turn = 0.33;          % enough to turn against the brake in the force rulesets
float: aim_tolerance = 3.0;         % degrees between the turret and the shot
float: reserve = 10.0;              % battery kept back after a shot
float: low_battery = battery_capacity / 4;
float: keep_out = 60.0;             % distance kept from the storm's edge and the zone's

output ["{\"steer\":", showJSON(steer), ",\"home\":", showJSON(home),
        ",\"range_step\":", showJSON(range_step), ",\"power\":", showJSON(power), ",\"cue_arc\":", showJSON(cue_arc),
        ",\"near_range\":", showJSON(near_range), ",\"far_range\":", showJSON(far_range),
        ",\"track_arc\":", showJSON(track_arc), ",\"search_arc\":", showJSON(search_arc),
        ",\"search_turn\":", showJSON(search_turn), ",\"aim_tolerance\":", showJSON(aim_tolerance),
        ",\"reserve\":", showJSON(reserve), ",\"low_battery\":", showJSON(low_battery),
        ",\"keep_out\":", showJSON(keep_out), "}\n"];
