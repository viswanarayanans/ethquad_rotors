#!/bin/sh

DESTDIR=/home/viswa/rotors_quad/ethquad_rotors/rotors_sim/build/nlopt/nlopt_install make install

cp -r /home/viswa/rotors_quad/ethquad_rotors/rotors_sim/build/nlopt/nlopt_install//home/viswa/rotors_quad/ethquad_rotors/rotors_sim/install/* /home/viswa/rotors_quad/ethquad_rotors/rotors_sim/devel/.private/nlopt/
