#!/bin/bash
cd client
npm install --legacy-peer-deps
npm install -g @angular/cli

cd ../server
npm install --legacy-peer-deps
npm install -g @nestjs/cli

cd ../project_ws
colcon build --cmake-args -DBUILD_TESTING=ON
source install/setup.sh

cd ..
docker compose up --build
