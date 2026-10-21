<...>
M623; end of "timelapse_record_flag"

M400 ; wait all motion done
M17 S
M17 Z0.4 ; lower z motor current to reduce impact if there is something in the bottom
G1 Z250 F600
G1 Z248
M400 P100
M17 R ; restore z current

M220 S100  ; Reset feedrate magnitude
<...>