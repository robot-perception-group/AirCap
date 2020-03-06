#!/bin/bash
TBOARDLOGS=$1
# >>> conda initialize >>>
# !! Contents within this block are managed by 'conda init' !!
__conda_setup="$('/home/${USER}/anaconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
 if [ $? -eq 0 ]; then
     eval "$__conda_setup"
 else
     if [ -f "/home/${USER}/anaconda3/etc/profile.d/conda.sh" ]; then
         . "/home/${USER}/anaconda3/etc/profile.d/conda.sh"
     else
         export PATH="/home/${USER}/anaconda3/bin:$PATH"
     fi
 fi
 unset __conda_setup
# <<< conda initialize <<<

conda activate spinningup
source ~/drl_ws/devel/setup.bash

tensorboard --logdir $TBOARDLOGS
