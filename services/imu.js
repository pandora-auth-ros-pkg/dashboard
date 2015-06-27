#!/usr/bin/env node

'use strict';


var Service = require('./service');


var imu = new Service({
  name: 'imu',
  topic: '/sensors/imu_rpy',
  op: 'subscribe',
  msgType: 'pandora_sensor_msgs/ImuRPY',
  serverTopic: 'sensors/imu'
}, function(msg) {
  var values = [
    {
      name: 'roll',
      value: msg.roll.toFixed(2)
    },
    {
      name: 'pitch',
      value: msg.pitch.toFixed(2)
    },
    {
      name: 'yaw',
      value: msg.yaw.toFixed(2)
    }
  ];

  return values;
});

imu.start();
