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
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/leekai/drone_ws/src/potential_planner"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/leekai/drone_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/leekai/drone_ws/install/lib/python2.7/dist-packages:/home/leekai/drone_ws/build/potential_planner/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/leekai/drone_ws/build/potential_planner" \
    "/usr/bin/python2" \
    "/home/leekai/drone_ws/src/potential_planner/setup.py" \
     \
    build --build-base "/home/leekai/drone_ws/build/potential_planner" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/leekai/drone_ws/install" --install-scripts="/home/leekai/drone_ws/install/bin"
