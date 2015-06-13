'use strict';

/*
 * Configuration shared among the services.
 */

var env = {};

env.getRosMasterURI = function() {
  return process.env.ROS_MASTER_URI;
};

env.getRosMasterIP = function() {
  return process.env.ROS_MASTER_URI.split('//')[1].split(':')[0];
};

env.rosBridgePort = 9090;

module.exports = env;
