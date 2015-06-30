#!/usr/bin/env node

'use strict';

var Service = require('./service');


var QRAlert = new Service({
  name: 'qrAlert',
  topic: '/data_fusion/alert_handler/qr_notification',
  op: 'subscribe',
  msgType: 'pandora_data_fusion_msgs/QrInfo',
  serverTopic: 'alert/qr'
}, function(msg) {
  var date = new Date(msg.timeFound.secs * 1000);

  return {
    id: msg.id,
    content: msg.content,
    probability: msg.probability.toFixed(2),
    timeFound: date.toString(),
    position: {
      frameID: msg.qrPose.header.frame_id,
      x: msg.qrPose.pose.position.x.toFixed(3),
      y: msg.qrPose.pose.position.y.toFixed(3),
      z: msg.qrPose.pose.position.z.toFixed(3)
    },
    qrFrameId: msg.qrFrameId
  };
});

QRAlert.start();
