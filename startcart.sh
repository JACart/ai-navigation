#!/bin/bash

#Function to start the cart from any directory and return to that directory
startcart () {
  cwd=$(pwd)
  cd $HOME && cd catkin_ws/src/ai-navigation
  ./run.sh
  cd cart_endpoints/scripts
  ./zed_passenger.py
  cd $HOME && cd $cwd
}