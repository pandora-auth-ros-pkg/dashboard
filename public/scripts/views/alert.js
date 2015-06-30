'use strict';

var Backbone = require('backbone');
var Dispatcher = require('../dispatcher');
var ioClient = require('../ros-events');

/**
 * View template.
 */

var alertTemplate = require('../templates/victim.alert.hbs');

/**
 * Alert Model
 */

var Alert = require('../models/alert');


var AlertView = Backbone.View.extend({

  el: '#alert-feed',

  template: alertTemplate,

  events: {
    'click .clean-alerts': 'cleanAlerts',
    'click .alert-remove': 'removeAlert',
    'click .alert-info': 'showAlertInfo'
  },

  initialize: function() {
    console.log('View alertfeed initialized');
    console.log(this);
    ioClient.on('web/alert/qr', this.appendQRAlert.bind(this));
  },

  activeAlerts: {},

  appendQRAlert: function(msg) {

    console.log('A QR alert has arrived.');

    // Create a new alert model for this alert.
    var alert = new Alert(msg);

    // Append the alert into the list.
    this.$el.append(this.template(alert.toJSON()));
  },

  cleanAlerts: function() {
  },


  render: function() {

    this.$el.html(this.template());

    return this;
  }

});


module.exports = AlertView;
