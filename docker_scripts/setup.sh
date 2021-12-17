# if GAZEBO_INSTALL_PREFIX is unset, fall back to
# configure-time value of CMAKE_INSTALL_PREFIX
installPrefix=${GAZEBO_INSTALL_PREFIX:-/usr}
if [ ! -d "${installPrefix}" ]; then
  echo "The install prefix ${installPrefix} does not exist."
  unset installPrefix
  return 1
elif [ ! -f "${installPrefix}/share/gazebo-11/setup.sh" ]; then
  echo "The install prefix ${installPrefix}/share/gazebo-11 does not contain setup.sh."
  unset installPrefix
  return 1
fi

export GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI:-http://localhost:11345}
export GAZEBO_MODEL_DATABASE_URI=
export GAZEBO_RESOURCE_PATH=${installPrefix}/share/gazebo-11:${GAZEBO_RESOURCE_PATH}
export GAZEBO_PLUGIN_PATH=${installPrefix}/lib/x86_64-linux-gnu/gazebo-11/plugins:${GAZEBO_PLUGIN_PATH}
export GAZEBO_MODEL_PATH=${installPrefix}/share/gazebo-11/models:${GAZEBO_MODEL_PATH}
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins
export OGRE_RESOURCE_PATH=/usr/lib/x86_64-linux-gnu/OGRE-1.9.0
