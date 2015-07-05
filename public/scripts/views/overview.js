'use strict';

/**
 * Dependencies
 */

var Backbone = require('backbone');

/**
 * Handlebars templates
 */

var overviewTemplate = require('../templates/overview.hbs');


var OverView = Backbone.View.extend({

  el: $('#main-dashboard-content'),

  overviewTemplate: overviewTemplate,

  initialize: function() {
    console.log('View initialized');
  },

  render: function() {
    this.$el.html(this.overviewTemplate({name: 'jon'}));
    return this;
  }

});

module.exports = OverView;
