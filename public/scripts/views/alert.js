'use strict';

var Backbone = require('backbone');
var ioClient = require('../ros-events');

/**
 * View template.
 */

var qrAlertTemplate = require('../templates/alert.qr.hbs');
var victimAlertTemplate = require('../templates/alert.victim.hbs');

/**
 * Alert Model
 */

var Alert = require('../models/alert');


var AlertView = Backbone.View.extend({

  el: '#alert-feed',

  qrTemplate: qrAlertTemplate,
  victimTemplate: victimAlertTemplate,

  events: {
    'click .clean-alerts': 'cleanAlerts',
    'click .alert-remove': 'removeAlert',
    'click .alert-info': 'showAlertInfo'
  },

  initialize: function() {
    console.log('View alertfeed initialized');
    console.log(this);
    ioClient.on('web/alert/qr', this.appendQRAlert.bind(this));
    ioClient.on('web/victim/alert', this.appendVictimAlert.bind(this));
  },

  qrs: [],
  victims: [],

  appendQRAlert: function(msg) {

    console.log('A QR alert has arrived.');

    // Create a new alert model for this alert.
    var alert = new Alert(msg);
    alert.set({'type': 'QR'});
    this.qrs.push(alert);

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

  appendVictimAlert: function(msg) {

    var alert = new Alert(msg);
    alert.set({'type': 'Victim'});
    alert.set({'id': this.victims.length + 1});
    this.victims.push(alert);

    this.$el.append(this.victimTemplate(alert.toJSON()));
  },

  cleanAlerts: function() {
  },


  render: function() {

    this.$el.html(this.qrTemplate());

    return this;
  }

});


module.exports = AlertView;
