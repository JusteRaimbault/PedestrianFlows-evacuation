extensions[table nw gis]


__includes[
  "setup.nls" 
  "utils.nls"
]



globals [
  
  ;;we will add to the impedance field some objective points and canonicals paths
  ;;to describe the environment more simply
  ;;let say we map a couple (objective point,origin region) to a list of successives positions
  ;;that will be calculated in cache at setup
  ;;can bring too much bord effects in a complicated env; have to check that
  trajectories
  
  ;;list of exits, considered as "objective points" in shortest path calculation heuristic
  objectives
  ;;same with origin points
  origins
  
  ;;remaining persons to evacuate
  remaining-pietons
  
  ;;Outputs:
  
  ;;count of waiting people: proxy of congestion
  ;;evacuation time
  ;;local density?
   
]



breed [pietons pieton]

;;breeds used for shortest path calculation
breed [abstract-nodes abstract-node]
undirected-link-breed [abstract-links abstract-link]

abstract-links-own [d]

patches-own [
  ;;models the resistance of terrain, although not directly formal impedance
  ;;\in [0;1], applied multiplically
  impedance
  
  ;;we won't handle infinite value for impedance
  is-wall?
  
  ;;initial position of agent
  is-initial?
  
  ;;source if creation of agents
  is-source?
]


pietons-own[
  ;;"maximal" mean-speed that won't vary during an evacuation
  ;;effective speed without interactions
  mean-speed
  
  ;;current speed
  ;;real speed of agent, obeys the equation
  ;;  ||v(t+1)||=||v(t)||||X(t+1)-X(t)||\alpha
  ;;  with \alpha = \frac{1}{v_{mean}\Delta_{t}}
  ;;(test if internally consistent)
  current-speed
  
  ;;formal trajectory calculed in cache
  ;;the guy will not follow it deterministically but will however
  ;;follow the global scheme
  trajectory
  current-target
  
  ;;var for outputs
  is-waiting?
  is-rerouted?
  
  ;;heuristics vars
  last-avoiding-wall
  last-avoiding-crowd
  avoid-crowd?
  cumul-stuck
  crowd-ahead
  
]








  
to new-random-pieton
  set shape pieton-shape set size patch-size / real-patch-size / 10 set color orange
  set mean-speed random-normal mean-mean-speed (sigma-mean-speed * mean-mean-speed / 100)
  set current-speed mean-speed
  move-to one-of patches with [not is-wall? and count pietons-here = 0]
  if trace-routes? [pen-down]
end

to new-pieton
  set shape pieton-shape set size patch-size / real-patch-size / 10 set color orange
  set mean-speed random-normal mean-mean-speed (sigma-mean-speed * mean-mean-speed / 100)
  set current-speed mean-speed
  set is-waiting? false
  set last-avoiding-wall 0 set last-avoiding-crowd 0 set avoid-crowd? false
  if trace-routes? [pen-down]
end


to build-trajectories
  
  ;;heuristic : calculate shortest path ; extract points from it
  
  ;;initialize the map
  set trajectories table:make
  set origins []
  
  ;;build set of origin points
  ;;heuristic with regular grid
  let x floor (min-pxcor + grid-size / 2) let y floor (min-pycor + grid-size / 2)
  repeat (floor (max-pxcor - min-pxcor) / grid-size) [
       repeat (floor (max-pycor - min-pycor) / grid-size) [
          if [not is-wall?] of patch x y [set origins lput patch x y origins]
          set y y + grid-size
       ]
       set y floor (min-pycor + grid-size / 2)
       set x x + grid-size
  ]
  
  ;;test
  ;;foreach origins [ask ? [set pcolor blue]]
  
  ;;calulate shortest paths and extract trajectories
  ;;step is grid size ?
  foreach objectives [
    let objective-node nobody ask ? [set objective-node one-of abstract-nodes in-radius 1]
    let objective ?
    foreach origins [
      ;;snapshot is done at the origin
      let traj []
      ask ? [let origin-node one-of abstract-nodes in-radius 1
        ask origin-node [
              set traj discrete-path-to objective-node
           ]
      ]
      
      ;;put the traj in the map
      table:put trajectories hashcode ? objective traj 
      
    ]
  ]
end

;;aux procedure to extract discrete path from travel
to-report discrete-path-to [objective]
  let traj []
  set traj fput patch-here traj
  let path nw:weighted-path-to objective "d"
  let nodes nw:turtles-on-weighted-path-to objective "d"
  if path != false [
     let cumulated-distance 0
     ;;extract "by hand" intermediate points
     foreach path [
       set cumulated-distance cumulated-distance + [d] of ?
       if cumulated-distance > subpaths-length [
         set traj lput [patch-here] of first nodes traj 
         set cumulated-distance 0 
       ]
       if length nodes > 0 [set nodes but-first nodes]
     ]
  ]
  set traj lput [patch-here] of objective traj
  report traj
end

to-report hashcode [o de]
  report (list [list pxcor pycor] of o [list pxcor pycor] of de)
end

to test-trajectories
  color-patches
  foreach table:get trajectories one-of table:keys trajectories [ask ? [set pcolor yellow]]
end


to attribute-trajectories
  ask pietons [
    find-trajectory
  ]
end

;;associated pieton procedure
to find-trajectory
  ;;randomly attribute an exit ; may be weighted in a future
    let dest one-of objectives
    let orig first sort-by [[distance myself] of ?1 < [distance myself] of ?2] origins
    set trajectory table:get trajectories hashcode orig dest
    ifelse length trajectory = 0 [die]
    [set current-target first trajectory
    set trajectory but-first trajectory]
end

;;test for pietons movements
;;people are ghosts in that test
;;structure of movement will be the same
to go
  
  ;;create new pietons if needed
  new-arrivants
  
  ask pietons [
    update-target
    update-direction
    move
    set color orange
  ]
  
  ;;reroute people who want to avoid the crowd
  reroute-some-people
  
  
  ;;DEBUG
  ;ask pietons with [count other pietons in-radius real-patch-size * time-step * current-speed / 4 > 0] [set color green]
  
  tick
end

to reroute-some-people
  ask pietons [set is-rerouted? false];debug prpose, delete in efficient run
  snapshot
  let changers (max-n-of (reroute-prop * (count pietons) / 100) pietons [crowd-ahead]) with [length trajectory > 0]
  ;;this list will contains O/D couples for which we have to recalculate trajectories
  ;;we update in map
  let new-trajectories []
  ask changers [
     let new-origin nobody let dest last trajectory
     ;;heads towards new dir, find origins
     set heading heading + toss-coin 90
     set new-origin one-of patches in-cone 8 60 with [member? self origins and distance myself > 5]
     if new-origin != nobody [set new-trajectories lput (list new-origin dest) new-trajectories set trajectory (list new-origin dest)]
  ]
  ;;recalculate, suppsoe to take density into account
  foreach new-trajectories [
    let o-patch first ? let dest-patch last ?
    let o nobody let dest nobody ask o-patch [set o one-of abstract-nodes in-radius 1] ask dest-patch [set dest one-of abstract-nodes in-radius 1]
     ask o [
       let traj discrete-path-to dest
       table:put trajectories hashcode o-patch dest-patch traj
     ]
  ]
  ;;reattribute traj to people
  ask changers with [length trajectory = 2][
    set trajectory table:get trajectories hashcode (first trajectory) (last trajectory)
    set current-target first trajectory
    set is-rerouted? true
  ]
  
end



to new-arrivants
  ;;here entry flow is not taken into account yet, max capacity ?
  if remaining-pietons > 0 [
    ask patches with [is-source? and count pietons-here = 0] [ if remaining-pietons > 0 [sprout-pietons 1 [new-pieton find-trajectory] set remaining-pietons remaining-pietons - 1]]
  ]
end



;;pieton procedure to update heading before move
to update-direction
  ;;heuristic: best direction minimize impedance in a given radius
  ;;take the mean with target direction
;  let h 0 let h2 0
;  if current-target != patch-here [set h towards current-target]
;  let patch-viewed sort-on [1 - impedance] patches in-cone cone-radius cone-angle with [self != [patch-here] of myself and impedance > 0]
;  if length patch-viewed != 0 [set h2 towards first patch-viewed]
;  set heading (h2 + h) / 2
  ;if current-target != patch-here [set heading towards current-target - 40 + random 80]
  ;if length trajectory > 0 [set heading towards last trajectory]
  ;set heading random 360
  
  ;;different prority orders
  ;;beware of cone radius and angle, parameters are essentials for the realistic aspect of the heuristic
  
  ;;still guys stuck in corners...
  ;;solution: angle a bit superior to 90 degrees?
  
  ;;if high local density of people should also minimize that density ?
  
  ;;first handle walls evitment
  ;;TODO memorize choice and don't do random when going along a wall
  set avoid-crowd? false
  let th 0
  if current-target != patch-here [set th towards current-target]
  
  let walls patches in-cone cone-radius cone-angle with [self != [patch-here] of myself and impedance = 0]
  ifelse count walls > 0 [

      let p patch (mean [pxcor] of walls) (mean [pycor] of walls) let h 0
      ifelse p = patch-here [set h heading][set h towards p]
      set heading ((direction (walls-eviting-angle + h) th) * walls-eviting-angle)

  ]
  
  ;;then treat "normal case"
  [if ticks - last-avoiding-crowd > 5 [ifelse current-target != patch-here [set heading towards current-target][ifelse length trajectory > 0 [set heading towards first trajectory][set heading heading - 90 + random 180]]]]
  
  ;;finally, correction to avoid "condensation"
  if count other pietons in-cone (current-speed * time-step * real-patch-size) 120 != 0 [set heading heading - 50 + random 100]
  
  ;;add other correction against high densities
  ;;and against "File effects"
  ;;unfortunately, due to the internal implementation of Dijsktra,
  ;;shortest path present recurrent patterns and
  ;;"highways" are created. Against that, go left or right if too much people ahead
  ;;(but greater cone of vision) !! small angle
  ;;also need to add a memory for that !
  let max-people (count patches in-cone overcrowding-radius overcrowding-angle)
  if max-people != 0 [
    set crowd-ahead (count pietons in-cone overcrowding-radius overcrowding-angle) / (count patches in-cone overcrowding-radius overcrowding-angle)
    ;lets try with "deterministic" reroutings
;    if crowd-ahead > overcrowding-threshold [
;      set heading heading + toss-coin overcrowding-angle
;      set last-avoiding-crowd ticks
;      set avoid-crowd? true
;    ]
  ]
;  
  
end

to-report toss-coin [value]
  ifelse random 2 = 0 [report value][report (- value)]
end

to-report direction [h1 h2]
  ifelse cos (h1 - h2) > 0 [report 1][report (-1)]
end

;;move procedure
;;speed is updated here
to move
  if cumul-stuck > 10 [die]
  set is-waiting? false
  let step current-speed * time-step * real-patch-size
  ifelse can-move-ahead? step[
    set cumul-stuck 0
    fd step * [impedance] of patch-here
  ][set is-waiting? true set cumul-stuck cumul-stuck + 1]
end


to-report can-move-ahead? [step]
  let p patch-ahead step if p = nobody [report false]
  report [not is-wall?] of p and count other pietons in-cone step 60 < impact-tolerance
end


;;test if current target has been "reached"
;;finish the travel if destination is reached
to update-target
  ifelse length trajectory > 0 [
    if current-target != patch-here and first trajectory != patch-here [
      if abs (towards current-target - towards first trajectory) > 90 [
        set current-target first trajectory
        set trajectory but-first trajectory
      ]
    ]
    if length trajectory > 0 
      [if current-target = patch-here or first trajectory = patch-here[
      set current-target first trajectory
      set trajectory but-first trajectory
      ]
    ]
  ][
    ;;just test if not far from destination
    if distance current-target < 2 [die]
  ]
end



;;exploration of linearity
to evac-time
  setup
  set-current-plot "evac-time"
  set-current-plot-pen "pen-0"
  set-plot-pen-mode 0
  set pietons-number 100
  repeat 20[
    setup-agents
    reset-ticks
    while [remaining-pietons > 0 or count pietons / pietons-number > 0.5][
      go
    ]
    plotxy pietons-number ticks
    set pietons-number pietons-number + 100
  ]

end




@#$#@#$#@
GRAPHICS-WINDOW
210
10
730
419
42
31
6.0
1
10
1
1
1
0
0
0
1
-42
42
-31
31
0
0
1
ticks
30.0

BUTTON
7
10
128
43
NIL
setup-random
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

SLIDER
1229
16
1401
49
mean-mean-speed
mean-mean-speed
1
10
1
0.1
1
NIL
HORIZONTAL

SLIDER
1231
54
1405
87
sigma-mean-speed
sigma-mean-speed
0
100
2
1
1
NIL
HORIZONTAL

SLIDER
1231
93
1403
126
pietons-number
pietons-number
0
3000
688
1
1
NIL
HORIZONTAL

SLIDER
1224
268
1396
301
grid-size
grid-size
1
world-width
2
1
1
NIL
HORIZONTAL

SLIDER
1049
17
1221
50
real-patch-size
real-patch-size
0
10
0.5
0.1
1
NIL
HORIZONTAL

SLIDER
1050
54
1222
87
time-step
time-step
0
60
0.5
0.1
1
NIL
HORIZONTAL

BUTTON
7
118
136
151
go
go\n;if count pietons with [count other pietons in-radius real-patch-size * time-step * current-speed / 4 > 0] > 0[\n; error \"ghosts !\"\n;]
T
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

SWITCH
1224
312
1365
345
trace-routes?
trace-routes?
1
1
-1000

BUTTON
133
11
207
44
NIL
setup-agents
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

SLIDER
1225
354
1397
387
cone-radius
cone-radius
1
20
1
1
1
NIL
HORIZONTAL

SLIDER
1226
391
1398
424
cone-angle
cone-angle
0
360
92
1
1
NIL
HORIZONTAL

MONITOR
15
209
90
254
remaining
remaining-pietons
17
1
11

BUTTON
7
47
73
80
NIL
setup
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

SLIDER
1235
132
1407
165
impact-tolerance
impact-tolerance
0
10
10
1
1
NIL
HORIZONTAL

CHOOSER
1223
467
1361
512
pieton-shape
pieton-shape
"person" "turtle"
1

SLIDER
1224
427
1396
460
walls-eviting-angle
walls-eviting-angle
0
180
136
1
1
NIL
HORIZONTAL

SLIDER
1072
355
1221
388
overcrowding-radius
overcrowding-radius
0
50
3
1
1
NIL
HORIZONTAL

SLIDER
1072
393
1219
426
overcrowding-angle
overcrowding-angle
0
180
31
1
1
NIL
HORIZONTAL

SLIDER
1073
432
1219
465
overcrowding-threshold
overcrowding-threshold
0
10
5.5
0.1
1
NIL
HORIZONTAL

SLIDER
912
16
1047
49
density-coef
density-coef
0
2
1.1
0.1
1
NIL
HORIZONTAL

INPUTBOX
1224
530
1358
590
impedance-file
../Data/impedanceGare.shp
1
0
String

INPUTBOX
1226
592
1358
652
obj-file
../Data/objectives.shp
1
0
String

SLIDER
1051
94
1223
127
initial-fill-prop
initial-fill-prop
0
100
50
1
1
NIL
HORIZONTAL

MONITOR
739
16
814
61
time (min)
ticks * time-step / 60
17
1
11

MONITOR
15
257
98
302
trajectories
table:length trajectories * 100 / (length origins * length objectives)
17
1
11

PLOT
740
74
900
194
Waiting people
NIL
NIL
0.0
10.0
0.0
10.0
true
false
"" ""
PENS
"default" 1.0 0 -16777216 true "" "if count pietons > 0 [plot 100 * count pietons with [is-waiting?] / count pietons]"

MONITOR
15
162
74
207
pietons
count pietons
17
1
11

SLIDER
1225
230
1397
263
subpaths-length
subpaths-length
0
10
5
1
1
NIL
HORIZONTAL

MONITOR
15
305
126
350
prop-overcr (%)
100 * count pietons with [avoid-crowd?] / count pietons
17
1
11

PLOT
745
206
945
356
evac-time
people number
time
0.0
10.0
0.0
10.0
true
false
"" ""
PENS
"pen-0" 1.0 0 -16777216 true "" ""

BUTTON
13
533
106
566
NIL
evac-time
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

SLIDER
1074
468
1216
501
reroute-prop
reroute-prop
0
100
13
1
1
NIL
HORIZONTAL

MONITOR
15
353
119
398
prop-rerouted
100 * count pietons with [is-rerouted?] / count pietons
17
1
11

@#$#@#$#@
## WHAT IS IT?

Simple implementation of pedestrian evacuation agent-based model

## HOW IT WORKS

see comments

## HOW TO USE IT

(how to use the model, including a description of each of the items in the Interface tab)

## THINGS TO NOTICE

(suggested things for the user to notice while running the model)

## THINGS TO TRY

(suggested things for the user to try to do (move sliders, switches, etc.) with the model)

## EXTENDING THE MODEL

(suggested things to add or change in the Code tab to make the model more complicated, detailed, accurate, etc.)

## NETLOGO FEATURES

(interesting or unusual features of NetLogo that the model uses, particularly in the Code tab; or where workarounds were needed for missing features)

## RELATED MODELS

(models in the NetLogo Models Library and elsewhere which are of related interest)

## CREDITS AND REFERENCES

(a reference to the model's URL on the web if it has one, as well as any other necessary credits, citations, and links)
@#$#@#$#@
default
true
0
Polygon -7500403 true true 150 5 40 250 150 205 260 250

airplane
true
0
Polygon -7500403 true true 150 0 135 15 120 60 120 105 15 165 15 195 120 180 135 240 105 270 120 285 150 270 180 285 210 270 165 240 180 180 285 195 285 165 180 105 180 60 165 15

arrow
true
0
Polygon -7500403 true true 150 0 0 150 105 150 105 293 195 293 195 150 300 150

box
false
0
Polygon -7500403 true true 150 285 285 225 285 75 150 135
Polygon -7500403 true true 150 135 15 75 150 15 285 75
Polygon -7500403 true true 15 75 15 225 150 285 150 135
Line -16777216 false 150 285 150 135
Line -16777216 false 150 135 15 75
Line -16777216 false 150 135 285 75

bug
true
0
Circle -7500403 true true 96 182 108
Circle -7500403 true true 110 127 80
Circle -7500403 true true 110 75 80
Line -7500403 true 150 100 80 30
Line -7500403 true 150 100 220 30

butterfly
true
0
Polygon -7500403 true true 150 165 209 199 225 225 225 255 195 270 165 255 150 240
Polygon -7500403 true true 150 165 89 198 75 225 75 255 105 270 135 255 150 240
Polygon -7500403 true true 139 148 100 105 55 90 25 90 10 105 10 135 25 180 40 195 85 194 139 163
Polygon -7500403 true true 162 150 200 105 245 90 275 90 290 105 290 135 275 180 260 195 215 195 162 165
Polygon -16777216 true false 150 255 135 225 120 150 135 120 150 105 165 120 180 150 165 225
Circle -16777216 true false 135 90 30
Line -16777216 false 150 105 195 60
Line -16777216 false 150 105 105 60

car
false
0
Polygon -7500403 true true 300 180 279 164 261 144 240 135 226 132 213 106 203 84 185 63 159 50 135 50 75 60 0 150 0 165 0 225 300 225 300 180
Circle -16777216 true false 180 180 90
Circle -16777216 true false 30 180 90
Polygon -16777216 true false 162 80 132 78 134 135 209 135 194 105 189 96 180 89
Circle -7500403 true true 47 195 58
Circle -7500403 true true 195 195 58

circle
false
0
Circle -7500403 true true 0 0 300

circle 2
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240

cow
false
0
Polygon -7500403 true true 200 193 197 249 179 249 177 196 166 187 140 189 93 191 78 179 72 211 49 209 48 181 37 149 25 120 25 89 45 72 103 84 179 75 198 76 252 64 272 81 293 103 285 121 255 121 242 118 224 167
Polygon -7500403 true true 73 210 86 251 62 249 48 208
Polygon -7500403 true true 25 114 16 195 9 204 23 213 25 200 39 123

cylinder
false
0
Circle -7500403 true true 0 0 300

dot
false
0
Circle -7500403 true true 90 90 120

face happy
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 255 90 239 62 213 47 191 67 179 90 203 109 218 150 225 192 218 210 203 227 181 251 194 236 217 212 240

face neutral
false
0
Circle -7500403 true true 8 7 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Rectangle -16777216 true false 60 195 240 225

face sad
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 168 90 184 62 210 47 232 67 244 90 220 109 205 150 198 192 205 210 220 227 242 251 229 236 206 212 183

fish
false
0
Polygon -1 true false 44 131 21 87 15 86 0 120 15 150 0 180 13 214 20 212 45 166
Polygon -1 true false 135 195 119 235 95 218 76 210 46 204 60 165
Polygon -1 true false 75 45 83 77 71 103 86 114 166 78 135 60
Polygon -7500403 true true 30 136 151 77 226 81 280 119 292 146 292 160 287 170 270 195 195 210 151 212 30 166
Circle -16777216 true false 215 106 30

flag
false
0
Rectangle -7500403 true true 60 15 75 300
Polygon -7500403 true true 90 150 270 90 90 30
Line -7500403 true 75 135 90 135
Line -7500403 true 75 45 90 45

flower
false
0
Polygon -10899396 true false 135 120 165 165 180 210 180 240 150 300 165 300 195 240 195 195 165 135
Circle -7500403 true true 85 132 38
Circle -7500403 true true 130 147 38
Circle -7500403 true true 192 85 38
Circle -7500403 true true 85 40 38
Circle -7500403 true true 177 40 38
Circle -7500403 true true 177 132 38
Circle -7500403 true true 70 85 38
Circle -7500403 true true 130 25 38
Circle -7500403 true true 96 51 108
Circle -16777216 true false 113 68 74
Polygon -10899396 true false 189 233 219 188 249 173 279 188 234 218
Polygon -10899396 true false 180 255 150 210 105 210 75 240 135 240

house
false
0
Rectangle -7500403 true true 45 120 255 285
Rectangle -16777216 true false 120 210 180 285
Polygon -7500403 true true 15 120 150 15 285 120
Line -16777216 false 30 120 270 120

leaf
false
0
Polygon -7500403 true true 150 210 135 195 120 210 60 210 30 195 60 180 60 165 15 135 30 120 15 105 40 104 45 90 60 90 90 105 105 120 120 120 105 60 120 60 135 30 150 15 165 30 180 60 195 60 180 120 195 120 210 105 240 90 255 90 263 104 285 105 270 120 285 135 240 165 240 180 270 195 240 210 180 210 165 195
Polygon -7500403 true true 135 195 135 240 120 255 105 255 105 285 135 285 165 240 165 195

line
true
0
Line -7500403 true 150 0 150 300

line half
true
0
Line -7500403 true 150 0 150 150

pentagon
false
0
Polygon -7500403 true true 150 15 15 120 60 285 240 285 285 120

person
false
0
Circle -7500403 true true 110 5 80
Polygon -7500403 true true 105 90 120 195 90 285 105 300 135 300 150 225 165 300 195 300 210 285 180 195 195 90
Rectangle -7500403 true true 127 79 172 94
Polygon -7500403 true true 195 90 240 150 225 180 165 105
Polygon -7500403 true true 105 90 60 150 75 180 135 105

plant
false
0
Rectangle -7500403 true true 135 90 165 300
Polygon -7500403 true true 135 255 90 210 45 195 75 255 135 285
Polygon -7500403 true true 165 255 210 210 255 195 225 255 165 285
Polygon -7500403 true true 135 180 90 135 45 120 75 180 135 210
Polygon -7500403 true true 165 180 165 210 225 180 255 120 210 135
Polygon -7500403 true true 135 105 90 60 45 45 75 105 135 135
Polygon -7500403 true true 165 105 165 135 225 105 255 45 210 60
Polygon -7500403 true true 135 90 120 45 150 15 180 45 165 90

sheep
false
15
Circle -1 true true 203 65 88
Circle -1 true true 70 65 162
Circle -1 true true 150 105 120
Polygon -7500403 true false 218 120 240 165 255 165 278 120
Circle -7500403 true false 214 72 67
Rectangle -1 true true 164 223 179 298
Polygon -1 true true 45 285 30 285 30 240 15 195 45 210
Circle -1 true true 3 83 150
Rectangle -1 true true 65 221 80 296
Polygon -1 true true 195 285 210 285 210 240 240 210 195 210
Polygon -7500403 true false 276 85 285 105 302 99 294 83
Polygon -7500403 true false 219 85 210 105 193 99 201 83

square
false
0
Rectangle -7500403 true true 30 30 270 270

square 2
false
0
Rectangle -7500403 true true 30 30 270 270
Rectangle -16777216 true false 60 60 240 240

star
false
0
Polygon -7500403 true true 151 1 185 108 298 108 207 175 242 282 151 216 59 282 94 175 3 108 116 108

target
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240
Circle -7500403 true true 60 60 180
Circle -16777216 true false 90 90 120
Circle -7500403 true true 120 120 60

tree
false
0
Circle -7500403 true true 118 3 94
Rectangle -6459832 true false 120 195 180 300
Circle -7500403 true true 65 21 108
Circle -7500403 true true 116 41 127
Circle -7500403 true true 45 90 120
Circle -7500403 true true 104 74 152

triangle
false
0
Polygon -7500403 true true 150 30 15 255 285 255

triangle 2
false
0
Polygon -7500403 true true 150 30 15 255 285 255
Polygon -16777216 true false 151 99 225 223 75 224

truck
false
0
Rectangle -7500403 true true 4 45 195 187
Polygon -7500403 true true 296 193 296 150 259 134 244 104 208 104 207 194
Rectangle -1 true false 195 60 195 105
Polygon -16777216 true false 238 112 252 141 219 141 218 112
Circle -16777216 true false 234 174 42
Rectangle -7500403 true true 181 185 214 194
Circle -16777216 true false 144 174 42
Circle -16777216 true false 24 174 42
Circle -7500403 false true 24 174 42
Circle -7500403 false true 144 174 42
Circle -7500403 false true 234 174 42

turtle
true
0
Polygon -10899396 true false 215 204 240 233 246 254 228 266 215 252 193 210
Polygon -10899396 true false 195 90 225 75 245 75 260 89 269 108 261 124 240 105 225 105 210 105
Polygon -10899396 true false 105 90 75 75 55 75 40 89 31 108 39 124 60 105 75 105 90 105
Polygon -10899396 true false 132 85 134 64 107 51 108 17 150 2 192 18 192 52 169 65 172 87
Polygon -10899396 true false 85 204 60 233 54 254 72 266 85 252 107 210
Polygon -7500403 true true 119 75 179 75 209 101 224 135 220 225 175 261 128 261 81 224 74 135 88 99

wheel
false
0
Circle -7500403 true true 3 3 294
Circle -16777216 true false 30 30 240
Line -7500403 true 150 285 150 15
Line -7500403 true 15 150 285 150
Circle -7500403 true true 120 120 60
Line -7500403 true 216 40 79 269
Line -7500403 true 40 84 269 221
Line -7500403 true 40 216 269 79
Line -7500403 true 84 40 221 269

wolf
false
0
Polygon -16777216 true false 253 133 245 131 245 133
Polygon -7500403 true true 2 194 13 197 30 191 38 193 38 205 20 226 20 257 27 265 38 266 40 260 31 253 31 230 60 206 68 198 75 209 66 228 65 243 82 261 84 268 100 267 103 261 77 239 79 231 100 207 98 196 119 201 143 202 160 195 166 210 172 213 173 238 167 251 160 248 154 265 169 264 178 247 186 240 198 260 200 271 217 271 219 262 207 258 195 230 192 198 210 184 227 164 242 144 259 145 284 151 277 141 293 140 299 134 297 127 273 119 270 105
Polygon -7500403 true true -1 195 14 180 36 166 40 153 53 140 82 131 134 133 159 126 188 115 227 108 236 102 238 98 268 86 269 92 281 87 269 103 269 113

x
false
0
Polygon -7500403 true true 270 75 225 30 30 225 75 270
Polygon -7500403 true true 30 75 75 30 270 225 225 270

@#$#@#$#@
NetLogo 5.0.2
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
default
0.0
-0.2 0 1.0 0.0
0.0 1 1.0 0.0
0.2 0 1.0 0.0
link direction
true
0
Line -7500403 true 150 150 90 180
Line -7500403 true 150 150 210 180

@#$#@#$#@
0
@#$#@#$#@
