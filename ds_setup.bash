#!/usr/bin/env bash

# "#!/bin/bash"
#set -e   # exit script if any statement returns a non-true return value



# ...aliasing dev.......................................................................................................
# ref:
# - https://www.baeldung.com/linux/bash-alias-with-parameters
# - https://unix.stackexchange.com/questions/3773/how-to-pass-parameters-to-an-alias

DS_PATH=$(sudo find / -name 'Dockerized-SNOW' -type d 2>/dev/null)

( \
  echo ""; \
  echo "# Dockerized-SNOW aliases"; \
  echo "export DS_PATH=$(sudo find / -name 'Dockerized-SNOW' -type d 2>/dev/null)"
  echo "alias ds_cd='cd $DS_PATH'"; \
  echo "alias ds_attach='cd $DS_PATH && bash ds_attach.bash'"; \
  echo "alias ds_instantiate_develop='cd $DS_PATH && bash ds_instantiate_develop.bash'"; \
  echo "alias ds_instantiate_deploy='cd $DS_PATH && bash ds_instantiate_deploy.bash'"; \
  echo "alias ds_build_dependencies='cd $DS_PATH && bash ds_build_dependencies.bash'"; \
  echo "alias ds_build_deploy='cd $DS_PATH && bash ds_build_deploy.bash'"; \
  echo "alias ds_build_develop='cd $DS_PATH && bash ds_build_develop.bash'"; \
  echo "alias ds_build_melodic_python3='cd $DS_PATH && bash ds_build_melodic_python3.bash'"; \
  echo "alias _ds_build_and_push_norlab_MPPI='cd $DS_PATH && bash _ds_build_and_push_norlab_MPPI.bash'"; \
  echo ""; \
) >> ~/.bashrc

# Source bashrc
source ~/.bashrc

# Load environment variable from file
set -o allexport; source ${DS_PATH}/ds.env; set +o allexport


# ...CUDA toolkit path..................................................................................................
# ref dusty_nv comment at https://forums.developer.nvidia.com/t/cuda-nvcc-not-found/118068
if ! command -v nvcc -V &> /dev/null; then
  # nvcc command not working
  echo -e "${DS_MSG_BASE} Fixing CUDA path for nvcc"

  ( \
  echo ""; \
  echo "# CUDA toolkit related"; \
  echo "# ref dusty_nv comment at"; \
  echo "#    https://forums.developer.nvidia.com/t/cuda-nvcc-not-found/118068"; \
  echo "export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}"; \
  echo "export LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"; \
  echo ""; \
  ) >> ~/.bashrc

  source ~/.bashrc && echo -e "${DS_MSG_DONE} CUDA path hack added to ~/.bashrc for nvcc"
fi

if [[ $(nvcc -V | grep 'nvcc: NVIDIA (R) Cuda compiler driver') == "nvcc: NVIDIA (R) Cuda compiler driver" ]]; then echo "> if TRUE"; else echo "> else FALSE"; fi
  echo -e "${DS_MSG_DONE} nvcc installed properly"
  nvcc -V
else
  echo -e "${DS_MSG_ERROR} Check your nvcc installation. It's NOT installed properly!"
fi


echo -e "${DS_MSG_DONE} Setup completed! New available alias:
  ds_cd
  ds_attach
  ds_instantiate_develop
  ds_instantiate_deploy
  ds_build_dependencies
  ds_build_deploy
  ds_build_develop
  ds_build_melodic_python3
"

# (Priority) todo:on task end >> delete next bloc ↓↓
echo -e "${DS_MSG_WARNING} TEST execute nvcc -V"
nvcc -V

