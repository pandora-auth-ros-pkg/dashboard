'use strict';

var Backbone = require('backbone');
var ioClient = require('../ros-events');

/**
 * View template.
 */

var qrAlertTemplate = require('../templates/alert.qr.hbs');

/**
 * Alert Model
 */

var Alert = require('../models/alert');


var AlertView = Backbone.View.extend({

  el: '#alert-feed',

  qrTemplate: qrAlertTemplate,

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
    alert.set({'type': 'QR'});

    // Append the alert into the list.
    this.$el.append(this.qrTemplate(alert.toJSON()));

    // Show stacked notification.
    new PNotify({
      title: 'A QR arrived',
      text: 'More info on the alerts section',
      hide: false,
      type: 'success',
      confirm: {
        confirm: true
      },
      buttons: {
        closer: false
      },
      history: {
        history: false
      }
    }).get().on('pnotify.confirm', function() {
      console.log('QR validated.');
    });
  },

  cleanAlerts: function() {
  },


  render: function() {

    this.$el.html(this.qrTemplate());

    return this;
  }

});


module.exports = AlertView;
