'use strict';

var Backbone = require('backbone');
var ioClient = require('../ros-events.js');
var Dispatcher = require('../dispatcher');

/**
 * Model for the IMU indicator.
 *
 * @param name String - name of the IMU.
 * @param roll Float - roll value.
 * @param pitch Float - pitch value.
 * @param yaw Float - pitch yaw.
 * @param topic String - socket.io topic to listen to for incoming data.
 * @param dispatcherTopic String - topic to notify the rest of the app.
 */

var IMU = Backbone.Model.extend({

  defaults: {
    values: [
      {
        name: 'roll',
        value: 0
      },
      {
        name: 'pitch',
        value: 0
      },
      {
        name: 'yaw',
        value: 0
      }
    ],
    name: 'IMU'
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
      _this.set({'values': msg});
    });
  },

  unsubscribe: function() {
    ioClient.removeListener(this.get('topic'));
  }

});

module.exports = IMU;
