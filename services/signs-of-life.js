#!/usr/bin/env node

'use strict';

var Service = require('./service');


var signsOfLife = new Service({
  name: 'signsOfLife',
  topic: '/data_fusion/signs_of_life',
  op: 'subscribe',
  msgType: 'pandora_data_fusion_msgs/VictimProbabilities',
  serverTopic: 'signsOfLife'
}, function(msg) {
  console.log(msg);
  return msg;
});

signsOfLife.start();
