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

echo_and_run cd "/home/quad/viswa_ws/Rotors_sim/src/rotors_evaluation"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/quad/viswa_ws/Rotors_sim/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/quad/viswa_ws/Rotors_sim/install/lib/python2.7/dist-packages:/home/quad/viswa_ws/Rotors_sim/build/rotors_evaluation/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/quad/viswa_ws/Rotors_sim/build/rotors_evaluation" \
    "/usr/bin/python" \
    "/home/quad/viswa_ws/Rotors_sim/src/rotors_evaluation/setup.py" \
    build --build-base "/home/quad/viswa_ws/Rotors_sim/build/rotors_evaluation" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/quad/viswa_ws/Rotors_sim/install" --install-scripts="/home/quad/viswa_ws/Rotors_sim/install/bin"