'use strict';

/**
 * Dependencies
 */

var gulp = require('gulp');
var jshint = require('gulp-jshint');
var sass = require('gulp-sass');
var browserify = require('browserify');
var handlebars = require('browserify-handlebars');

/**
 * Utilities
 */

var source = require('vinyl-source-stream');
var buffer = require('vinyl-buffer');
var sourcemaps = require('gulp-sourcemaps');
var gutil = require('gulp-util');
var uglify = require('gulp-uglify');

/**
 * Project structure
 */

var settings = {
  public: {
    root: 'public/',
    index: 'public/index.html',
    js: {
      root: './public/scripts/**/*.js',
      entry: './public/scripts/app.js/'
    },
    styles: {
      root: './public/styles/**/*.scss',
      entry: './public/styles/main.scss'
    }
  },
  server: {
    services: './services/**/*.js',
    utils: './utils/**/*.js',
    app: 'app.js',
    bin: './bin/www'

  },
  build: {
    root: './public/build/',
    all: './public/build/*',
    js: './public/build/bundle.js',
    css: './public/build/main.css',
  }
};


/**
 * Gulp Tasks
 */

gulp.task('lint', function() {
  return gulp.src([settings.public.js.root,
                   settings.server.services,
                   settings.server.utils,
                   settings.server.bin,
                   settings.server.app])
             .pipe(jshint())
             .pipe(jshint.reporter('jshint-stylish'));
});

gulp.task('bundle', function() {
  var b = browserify({
    entries: settings.public.js.entry,
    debug: true,
    transform: [handlebars]
  });

  return b.bundle()
          .pipe(source('bundle.js'))
          .pipe(buffer())
          .pipe(sourcemaps.init({loadMaps: true}))
          .pipe(uglify())
          .on('error', gutil.log)
          .pipe(gulp.dest(settings.build.root));
});

gulp.task('sass', function() {
  gulp.src(settings.public.styles.entry)
      .pipe(sass())
      .pipe(gulp.dest(settings.build.root));
});

gulp.task('build', ['bundle', 'sass'], function() {
  console.log('App built.');
});

gulp.task('watch', function() {
  gulp.watch([settings.public.styles.root], ['sass']);
  gulp.watch([settings.public.js.root], ['lint']);
  gulp.watch([settings.server.services], ['lint']);
  gulp.watch([settings.server.utils], ['lint']);
  gulp.watch([settings.server.app], ['lint']);
  gulp.watch([settings.server.bin], ['lint']);
});

gulp.task('default' , ['watch'], function() {
});
