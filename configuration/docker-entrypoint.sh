#!/bin/bash

# make bash scripts executalbe by the (non-root) user
chmod a+x /home/isa_afa_3_ws/scripts/00_init.bash
chmod a+x /home/isa_afa_3_ws/scripts/01_clear_workspace.bash
chmod a+x /home/isa_afa_3_ws/scripts/10_rebuild.bash

# build the workspace
echo 'Starting to build workspace from docker-entrypoint'
bash /home/isa_afa_3_ws/scripts/00_init.bash

# let container open
echo 'container started'
tail -F /dev/null
