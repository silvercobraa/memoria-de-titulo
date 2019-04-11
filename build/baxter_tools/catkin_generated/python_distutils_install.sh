#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/silvercobraa/ros_ws/src/baxter_tools"

# snsure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/silvercobraa/ros_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/silvercobraa/ros_ws/install/lib/python2.7/dist-packages:/home/silvercobraa/ros_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/silvercobraa/ros_ws/build" \
    "/usr/bin/python" \
    "/home/silvercobraa/ros_ws/src/baxter_tools/setup.py" \
    build --build-base "/home/silvercobraa/ros_ws/build/baxter_tools" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/silvercobraa/ros_ws/install" --install-scripts="/home/silvercobraa/ros_ws/install/bin"
