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

echo_and_run cd "/home/zhang/catkin_learn/src/geometry2/tf2_kdl"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/zhang/catkin_learn/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/zhang/catkin_learn/install/lib/python2.7/dist-packages:/home/zhang/catkin_learn/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/zhang/catkin_learn/build" \
    "/usr/bin/python" \
    "/home/zhang/catkin_learn/src/geometry2/tf2_kdl/setup.py" \
    build --build-base "/home/zhang/catkin_learn/build/geometry2/tf2_kdl" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/zhang/catkin_learn/install" --install-scripts="/home/zhang/catkin_learn/install/bin"
