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

echo_and_run cd "/home/fast/rocon_ws/src/rocon_devices/rocon_hue"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/fast/rocon_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/fast/rocon_ws/install/lib/python3/dist-packages:/home/fast/rocon_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/fast/rocon_ws/build" \
    "/usr/bin/python3" \
    "/home/fast/rocon_ws/src/rocon_devices/rocon_hue/setup.py" \
     \
    build --build-base "/home/fast/rocon_ws/build/rocon_devices/rocon_hue" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/fast/rocon_ws/install" --install-scripts="/home/fast/rocon_ws/install/bin"
