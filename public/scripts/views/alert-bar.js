'use strict';

var Backbone = require('backbone');
var Dispatcher = require('../dispatcher');

/**
 * View template.
 */

var alertTemplate = require('../templates/victim.alert.hbs');
var singleAlert = require('../templates/victim.notification.hbs');


var AlertFeedView = Backbone.View.extend({

  el: '.alert-feed',

  template: alertTemplate,
  notificationTemplate: singleAlert,

  events: {
    'click .clean-alerts': 'cleanAlerts'
  },

  initialize: function() {
    console.log('View alertfeed initialized');
    Dispatcher.on('alert:victim', this.addVictimAlert, this);
  },

  cleanAlerts: function() {
    console.log('Clean alerts');
    this.$('ul').children().each(function(number, item) {
      if ($(this).hasClass('victim-alert')) {
        $(this).remove();
      }
    });
    this.$('.badge').html(0);
  },

  addVictimAlert: function(model) {
    console.log('Add victim alert.');

    var context = this.notificationTemplate(model);
    // Append a victim notification.
    this.$('ul.dropdown-menu').append(context);
    this.$('ul.dropdown-menu li.no-alerts').remove();

    // Increase by one the number of notifications.
    $('.alert-feed .badge').text(Number($('.alert-feed .badge').text()) + 1);
  },

  render: function() {

    this.$el.html(this.template());

    return this;
  }

});


module.exports = AlertFeedView;
