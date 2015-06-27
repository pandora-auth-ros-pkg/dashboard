'use strict';

var Backbone = require('backbone');
var ioClient = require('../ros-events.js');
var Dispatcher = require('../dispatcher');


/**
 * Model for the Sonar indicator.
 *
 * @param name String - name of the IMU.
 * @param right_range - range from the right sonar.
 * @param left_range - range from the left sonar.
 * @param topic String - socket.io topic to listen to for incoming data.
 * @param dispatcherTopic String - topic to notify the rest of the app.
 */

var Sonar = Backbone.Model.extend({

  defaults: {
    left_sensor: 0,
    right_sensor: 0,
    name: 'Sonar'
  },

  /**
   * Register events on the event dispatcher.'
   */

  initialize: function() {
    var _this = this;
    this.set({'dispatcherTopic': this.get('name').toLowerCase() + ':change'});

    /**
     * Notify other parts of the app for the imu change.
     */

    this.on('change', function() {
      Dispatcher.trigger(_this.get('dispatcherTopic'), _this.attributes);
    });
  },

  /**
   * Listen for incoming data from the socket.io server.
   */

  subscribe: function(topic) {
    this.set({'topic': topic || this.get('topic')});
    console.log('Listening on topic ' + this.get('topic'));

    var _this = this;
    ioClient.on(this.get('topic'), function(msg) {
      if (msg.right === null) {
        _this.set({'left_sensor': msg.left.toFixed(2)});
      } else {
        _this.set({'right_sensor': msg.right.toFixed(2)});
      }
    });
  },

  unsubscribe: function() {
    ioClient.removeListener(this.get('topic'));
  }

});

module.exports = Sonar;
