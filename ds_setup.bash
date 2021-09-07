#!/bin/bash

# ...aliasing dev.......................................................................................................
# ref:
# - https://www.baeldung.com/linux/bash-alias-with-parameters
# - https://unix.stackexchange.com/questions/3773/how-to-pass-parameters-to-an-alias

#DS_PATH=$(sudo find / -name 'Dockerized-SNOW' -type d 2>/dev/null)
#echo "export DS_PATH=$(sudo find / -name 'Dockerized-SNOW' -type d 2>/dev/null)" >> ~/.bashrc
#
#alias ds_cd="cd $DS_PATH"
#alias ds_attach="cd $DS_PATH && bash ds_attach.bash"
#alias ds_instantiate_develop="cd $DS_PATH && bash ds_instantiate_develop.bash"
#alias ds_instantiate_deploy="cd $DS_PATH && bash ds_instantiate_deploy.bash"
#alias ds_build_dependencies="cd $DS_PATH && bash ds_build_dependencies.bash"
#alias ds_build_deploy="cd $DS_PATH && bash ds_build_deploy.bash"
#alias ds_build_develop="cd $DS_PATH && bash ds_build_develop.bash"
#alias ds_build_melodic_python3="cd $DS_PATH && bash ds_build_melodic_python3.bash"
#

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
  echo ""; \
) >> ~/.bashrc
