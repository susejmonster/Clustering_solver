import processing.core.*;
import processing.data.*;
import processing.event.*;
import processing.opengl.*;

import java.util.HashMap;
import java.util.ArrayList;
import java.io.File;
import java.io.BufferedReader;
import java.io.PrintWriter;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.IOException;

int lasttime = 0;
float mouse_xrad = 5;
float mouse_yrad = 5;
Flock flock;

void setup() {
  size(540, 540); // Moved from settings() back to setup() where it belongs in .pde
  flock = new Flock();
  
  // Add an initial set of boids into the system
  for (int i = 0; i < 200; i++) {
    Boid b = new Boid(width/2 + random(0,75), height/2 + random(0,75));
    flock.addBoid(b);
  }
}

void draw() {
  background(255);
  flock.run();
}

// Add a new boid into the System
void mouseDragged() {
  flock.addBoid(new Boid(mouseX, mouseY));
}

void mouseWheel(MouseEvent event) {
  float e = event.getCount();
  println(e);
  if (e > 0) {
    mouse_xrad = mouse_xrad + 1;
    mouse_yrad = mouse_yrad + 1;
  }
  if (e < 0) {
    mouse_xrad = mouse_xrad - 1;
    mouse_yrad = mouse_yrad - 1;
  }
}


