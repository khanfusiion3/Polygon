const MAX_POLY_VERTEX_COUNT = 64;

class Polygon extends Shape {
  constructor(verts, hh) {
    super();
    this.vertices = Vec2.arrayOf(MAX_POLY_VERTEX_COUNT);
    this.normals = Vec2.arrayOf(MAX_POLY_VERTEX_COUNT);
    if (verts instanceof Array) {
      this.set(verts);
    } else if (verts && hh) {
      this.setBox(verts, hh);
    }
  }
  clone() {
    const p = new Polygon();
    p.u.set(this.u);
    for (let i = 0; i < this.vertices.length; ++i) {
      p.vertices[i].set(this.vertices[i]);
      p.normals[i].set(this.normals[i]);
    }
    return p;
  }
  initialize() {
    return this.computeMass(1);
  }
  computeMass(density) {
    const c = new Vec2(0, 0);
    let area = 0;
    let I = 0;
    const k_inv3 = 1 / 3;
    for (let i = 0; i < this.vertices.length; ++i) {
      const p1 = this.vertices[i];
      const p2 = this.vertices[(i + 1) % this.vertices.length];
      const D = p1.crossV(p2);
      const triangleArea = 0.5 * D;
      area += triangleArea;
      const weight = triangleArea * k_inv3;
      c.addsi(p1, weight);
      c.addsi(p2, weight);
      const intx2 = p1.x * p1.x + p2.x * p1.x + p2.x * p2.x;
      const inty2 = p1.y * p1.y + p2.y * p1.y + p2.y * p2.y;
      I += (0.25 * k_inv3 * D) * (intx2 + inty2);
    }
    c.muli(1 / area);
    for (let i = 0; i < this.vertices.length; ++i) {
      this.vertices[i].subi(c);
    }
    this.body.mass = density * area;
    this.body.invMass = this.body.mass !== 0 ? 1 / this.body.mass : 0;
    this.body.inertia = I * density;
    this.body.invInertia = this.body.inertia !== 0 ? 1 / this.body.inertia : 0;
    return this;
  }
  setOrient(radians) {
    this.u.set(radians);
  }
  getType() {
    return Type.Poly;
  }
  setBox(hw, hh) {
    this.vertices = Vec2.arrayOf(4);
    this.vertices[0].set(-hw, -hh);
    this.vertices[1].set(hw, -hh);
    this.vertices[2].set(hw, hh);
    this.vertices[3].set(-hw, hh);
    this.normals = Vec2.arrayOf(4);
    this.normals[0].set(0, -1);
    this.normals[1].set(1, 0);
    this.normals[2].set(0, 1);
    this.normals[3].set(-1, 0);
    return this;
  }
  set(verts) {
    let rightMost = 0;
    let highestXCoord = verts[0].x;
    for (let i = 1; i < verts.length; ++i) {
      const x = verts[i].x;
      if (x > highestXCoord) {
        highestXCoord = x;
        rightMost = i;
      } else if (x === highestXCoord) {
        if (verts[i].y < verts[rightMost].y) {
          rightMost = i;
        }
      }
    }
    const hull = new Array(MAX_POLY_VERTEX_COUNT);
    let outCount = 0;
    let indexHull = rightMost;
    for (;;) {
      hull[outCount] = indexHull;
      let nextHullIndex = 0;
      for (let i = 1; i < verts.length; ++i) {
        if (nextHullIndex === indexHull) {
          nextHullIndex = i;
          continue;
        }
        const e1 = verts[nextHullIndex].clone().subi(verts[hull[outCount]]);
        const e2 = verts[i].clone().subi(verts[hull[outCount]]);
        const c = e1.crossV(e2);
        if (c < 0) {
          nextHullIndex = i;
        }
        if (c === 0 && e2.lengthSq() > e1.lengthSq()) {
          nextHullIndex = i;
        }
      }
      ++outCount;
      indexHull = nextHullIndex;
      if (nextHullIndex === rightMost) {
        this.vertices.length = outCount;
        break;
      }
    }
    for (let i = 0; i < this.vertices.length; ++i) {
      this.vertices[i] = new Vec2(verts[hull[i]]);
    }
    for (let i = 0; i < this.vertices.length; ++i) {
      const face = this.vertices[(i + 1) % this.vertices.length].clone().subi(this.vertices[i]);
      this.normals[i] = new Vec2(face.y, -face.x);
      this.normals[i].normalize();
    }
  }
  getSupport(dir) {
    let bestProjection = -Number.MAX_VALUE;
    let bestVertex = null;
    for (let i = 0; i < this.vertices.length; ++i) {
      const v = this.vertices[i];
      const projection = v.dot(dir);
      if (projection > bestProjection) {
        bestVertex = v;
        bestProjection = projection;
      }
    }
    return bestVertex;
  }
}
