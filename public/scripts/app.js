'use strict';

/**
 * External dependencies
 */

var jquery = require('jquery');
var slideout = require('slideout');
var Backbone = require('backbone');

// Export jQuery to the global namespace.
window.$ = jquery;
window.jQuery = jquery;

// Slider initialization.
window.Slideout = slideout;
window.slider = new window.Slideout({
  'panel': document.getElementById('main-content'),
  'menu': document.getElementById('slide-menu'),
  'padding': 256,
  'tolerance': 70,
  'duration': 200
});

/**
 * App modules
 */

var MainRouter = require('./routes/main.router');

var router = new MainRouter();

Backbone.history.start();
