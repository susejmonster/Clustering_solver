class Boid {
  PVector position;
  PVector velocity;
  PVector acceleration;
  float r;
  float maxforce;    // Maximum steering force
  float maxspeed;    // Maximum speed
  int curr_t;
  int col;
  int size;//changed
  public int bernoulli() {
    float p = random(1.0); 
  
    if (p > 0.5) {
      return 10;
    } else {
      return 1;
    }
  }

  Boid(float x, float y) { //constructor
    acceleration = new PVector(0, 0);
    velocity = new PVector(random(-1, 1), random(-1, 1));
    position = new PVector(x, y);
    r = 5.0;
    maxspeed = 3;
    maxforce = 0.05;
    //col = color(175);//here
    size = bernoulli();
  }

  void run(ArrayList<Boid> boids) { 
    flock(boids);
    curr_t = millis();
    update();
    borders();
    render();
    MPCD(boids);
    //validate();
  }

  void applyForce(PVector force) {
    acceleration.add(force); 
  }

  void flock(ArrayList<Boid> boids) {
    PVector sep = separate(boids);   // Separation
    PVector ali = align(boids);      // Alignment
    PVector coh = cohesion(boids);   // Cohesion
    PVector inertia = Momentum(boids); //inertia
    //void MPCD = MPCD(boids); //solvent to solvent rotation

    sep.mult(1.0);
    ali.mult(1.0);
    coh.mult(1.0);
    //if boids.size[i]==1->solvent: apply rotation based on velocity of cm of bounding box
    
    //if boids.size[i]==0->active: apply LJ seperation force
    applyForce(sep);
    //applyForce(MPCD);
    applyForce(ali);
    applyForce(coh);//convert to wca between all active to solute particles
    applyForce(inertia);
  }

  void update() {
    velocity.add(acceleration);
    velocity.limit(maxspeed);
    position.add(velocity);
    acceleration.mult(0);
  }

  PVector seek(PVector target) {
    PVector desired = PVector.sub(target, position);  
    desired.normalize();
    desired.mult(maxspeed);
    
    PVector steer = PVector.sub(desired, velocity);
    steer.limit(maxforce);  
    return steer;
  }

  void render() {
    float theta = velocity.heading() + radians(90);
    fill(col);
    stroke(0);
    pushMatrix();
    translate(position.x, position.y);
    rotate(theta);
    beginShape(TRIANGLES);
    vertex(0, -r*2);
    vertex(-r, r*2);
    vertex(r, r*2);
    endShape();
    popMatrix();
  }

  void borders() {
    if (position.x < -r) position.x = width+r;
    if (position.y < -r) position.y = height+r;
    if (position.x > width+r) position.x = -r;
    if (position.y > height+r) position.y = -r;
  }

  PVector separate(ArrayList<Boid> boids) {
    float desiredseparation = 25.0;
    //ur = 4eps[(sigma/r)^12 - (sigma/r)^6] 
    //sigma = particle diameter, eps = interaction strength
    PVector steer = new PVector(0, 0, 0);
    int count = 0;
    for (Boid other : boids) {
      float d = PVector.dist(position, other.position);
      if ((d > 0) && (d < desiredseparation)) {
        PVector diff = PVector.sub(position, other.position);
        diff.normalize();
        diff.div(d);        
        steer.add(diff);
        count++;            
      }
    }
    if (count > 0) {
      steer.div((float)count);
    }
    if (steer.mag() > 0) {
      steer.normalize();
      steer.mult(maxspeed);
      steer.sub(velocity);
      steer.limit(maxforce);
    }
    return steer;
  }
  //solute to solute interaction
  void MPCD(ArrayList<Boid> boids){
    //get time

    //get velocity of particle
    PVector curr_vel = velocity;
    if(this.size<=1.0){
      //get boids of size == 0
      for(Boid other : boids){
        //edge case
        if(other == this){
          continue;
        }
        //draw bounding box
      boolean xOverlap = this.position.x < other.position.x + 5.0 && 
                         this.position.x + 5.0 > other.position.x;
                         
      boolean yOverlap = this.position.y < other.position.y + 5.0 && 
                         this.position.y + 5.0 > other.position.y;
        if(xOverlap && yOverlap){
          pushStyle(); 
          fill(255, 0, 0);      
          textSize(20);         
          textAlign(CENTER, TOP); 
          text("Collision: " + millis(), width / 2, 40); 
          popStyle();
        } 
      }
    }
    //matrix->
    //apply rotation 
    //return steer;
  }
  PVector align(ArrayList<Boid> boids) {
    float neighbordist = mouse_xrad;
    PVector V_T = new PVector(0, 0);
    int count = 0;
    float eta = random(0,1);
    PVector noise = PVector.random2D();
    float magnitude = 0.1 * (eta - 0.5); 
    noise.setMag(magnitude);
    
    for (Boid other : boids) {
      float d = PVector.dist(position, other.position);
      if ((d > 0) && (d < neighbordist)) {
        PVector v_t = PVector.fromAngle(other.velocity.heading()); 
        V_T.add(v_t);
        V_T.add(noise);
        count++;
      }
    }
    if (count > 0) {
      float dt = (curr_t - lasttime) / 1000.0;
      V_T.div((float)count); 
      V_T.mult(dt);
      V_T.normalize();
      V_T.mult(maxspeed); 
      PVector steer = PVector.sub(V_T, velocity);
      steer.limit(maxforce);
      return steer;
    } else {
      return new PVector(0, 0);
    }
  }
  //validate(){
  //}
  PVector cohesion(ArrayList<Boid> boids) {
    float neighbordist = 0;
    PVector sum = new PVector(0, 0);   
    int count = 0;
    for (Boid other : boids) {
      float d = PVector.dist(position, other.position);
      if ((d > 0) && (d < neighbordist)) {
        sum.add(other.position); 
        count++;
      }
    }
    if (count > 0) {
      sum.div(count);
      return seek(sum);  
    } else {
      return new PVector(0, 0);
    }
  }
  
public PVector Momentum(ArrayList<Boid> boids) {
  float neighbordist = mouse_xrad; 
  PVector desiredVelocitySum = new PVector(0, 0);
  int count = 0;
  
  
  float myMass = this.r; 
  if (myMass <= 0) myMass = 1.0; 

  for (Boid other : boids) {
    float d = PVector.dist(this.position, other.position);
    if ((d > 0) && (d < neighbordist)) {
      PVector neighborMomentum = PVector.mult(other.velocity, other.r);
      PVector matchedVelocity = PVector.div(neighborMomentum, myMass);
      desiredVelocitySum.add(matchedVelocity);
      count++;
    }
  }

  if (count > 0) {
    // Average all the desired velocities from the neighbors
    desiredVelocitySum.div((float)count);
    desiredVelocitySum.limit(maxspeed);
    PVector steer = PVector.sub(desiredVelocitySum, this.velocity);
    steer.limit(maxforce);  
    
    return steer;
  } else {
    // Return an empty vector if there are no neighbors
    return new PVector(0, 0);
  }
}
  PVector view(ArrayList<Boid> boids) {
    float sightDistance = mouse_xrad;
    float periphery = 2*PI;

    for (Boid other : boids) {
      PVector comparison = PVector.sub(other.position, position);
      float d = PVector.dist(position, other.position);
      float diff = PVector.angleBetween(comparison, velocity);

      if (diff < periphery && d > 0 && d < sightDistance) {
        other.highlight();
      }
    }

    float currentHeading = velocity.heading();
    pushMatrix();
    translate(position.x, position.y);
    rotate(currentHeading);
    fill(0, 100);
    arc(0, 0, sightDistance*2, sightDistance*2, -periphery, periphery);
    popMatrix();

    return new PVector();
  }

  void highlight() {
    col = color(255, 0, 0);
  }
}