#!/bin/bash


echo -e "
\033[1;2m

               .|'''.|
               ||..  '
               ''|||.            \033[0m \033[1;37m
•••·· ·· · Dockerized-SNOW ··· ·••$(printf '·%.s' {1..$(( $(tput cols) - 41 ))}) ·· ·\033[0m\033[1;2m
                NorLab
              .     '||
              |'....|'
\n\033[0m \033[2;37m
       https://norlab.ulaval.ca
    https://redleader962.github.io

\033[0m
"

echo -e "
\033[2;37m \n
                        (remote dev)
                   ˛.••·Jetson arm64
   ROS··••NVIDIA-docker
                      ˙'··••x86
                         (local dev)
\n\033[0m
"

# ======================================================================================================================

echo -e "
\033[1;2m


                   .|'''.|  '|.   '|'  ..|''||   '|| '||'  '|'
                   ||..  '   |'|   |  .|'    ||   '|. '|.  .'
                    ''|||.   | '|. |  ||      ||   ||  ||  |
                  .     '||  |   |||  '|.     ||    ||| |||
                  |'....|'  .|.   '|   ''|...|'      |   |

                               (Dockerized-SNOW)

                https://github.com/RedLeader962/Dockerized-SNOW
                           https://norlab.ulaval.ca

\033[0m
"

# ======================================================================================================================

echo -e "
\033[1;2m


               .|'''.|
               ||..  '
               ''|||.            \033[0m \033[1;37m
•••·· ·· · Dockerized-SNOW ··· ·••$(printf '·%.s' {1..$(( $(tput cols) - 41 ))}) ·· ·\033[0m\033[1;2m
                NorLab
              .     '||
              |'....|'
\n\033[0m \033[2;37m
       https://norlab.ulaval.ca
    https://redleader962.github.io


\033[0m
"

# Print a straight line of terminal window lenght
# Ref: https://stackoverflow.com/questions/42762643/draw-a-horizontal-line-from-dash-character-occupying-the-full-width-of-a-termina/42762743
#
# $ printf '─%.s' {1..$(tput cols)}
# ──────────────────────────────────────────────────────────────────────────────────────

export RED=$(tput setaf 1 :-"" 2>/dev/null)
export RESET=$(tput sgr0 :-"" 2>/dev/null)
echo $RED; printf -- "-%.0s" $(seq $(tput cols)); echo $RESET




echo -e "
\033[1;2m

               .|'''.|
               ||..  '
               ''|||.
\033[1;7m            Dockerized-SNOW            \033[0m\033[1;2m
                NorLab
              .     '||
              |'....|'

\033[0m
\033[2;37m
                        (remote dev)
                   ˛.••·Jetson arm64
   ROS··••NVIDIA-docker
                      ˙'··••x86
                         (local dev)


   › https://norlab.ulaval.ca
   › https://redleader962.github.io

\033[0m
"

echo -e "
\033[1;2m

          Warthog‹‹››Jetson‹‹››NVIDIA-docker‹‹››ROS

          Warthog·••·Jetson·••·NVIDIA-docker·••·ROS

          Warthog·••·Jetson·••·NVIDIA-docker·••·ROS

          Warthog··••··Jetson··••··NVIDIA-docker··••··ROS


                              ˛.••· arm64 (Jetson remote dev)
          ROS ··••·· NVIDIA-docker
                              ˙'••· x86 (local dev)


         ˙˚˚˙ºº˛˛``´´´”„¨¨¨°˚˚ª¢

          ·•·        ·••·       ··••··

          ‹‹››        «‹›»        ‹«‹›»›

\033[0m
"