const XY = (v1, v2) => {
  const x = v1 || 0
  const y = v2 || 0
  const cal_len = (a, b) => Math.sqrt(a * a + b * b)
  const len = cal_len(x, y)
  
  const sub = ({ x: a, y: b }) => XY(x - a, y - b)
  const add = ({ x: a, y: b }) => XY(x + a, y + b)
  const mul = ({ x: a, y: b }) => XY(x * a, y * b)
  const div = ({ x: a, y: b }) => XY(x / a, y / b)
  const mul_n = n => XY(x * n, y * n)
  const div_n = n => XY(x / n, y / n)
  
  const distance = ({ x: a, y: b }) => cal_len(x - a, y - b)
  const normalize = () => ((len > 0) ? XY(x / len, y / len) : XY())
  
  return ({
    x, y, len,
    sub, add, mul, div,
    mul_n, div_n,
    normalize,
    distance,
  })
}

const Point = (x, y, in_fixed) => {
  let pos = XY(x, y)
  let pre = XY(x, y)
  let acc = XY()
  const fixed = in_fixed
  
  const move = v => {
    if (fixed) {
      return
    }
    pos = pos.add(v)
  }
  
  const add_force = v => {
    if (fixed) {
      return
    }
    acc = acc.add(v)
  }
  
  const update = delta => {
    if (fixed) {
      return
    }
    
    const old_pos = pos
    
    delta *= delta
    acc = acc.mul_n(delta)
    pos = pos.add(pos.sub(pre).add(acc))
    acc = XY()
    
    pre = old_pos
  }
  
  const check_walls = (x, y, w, h) => {
/*
Point.prototype.check_walls = function(x, y, w, h) {

    this.pos.x = Math.max(x + 1, Math.min(w - 1, this.pos.x));
    this.pos.y = Math.max(y + 1, Math.min(h - 1, this.pos.y));

    if (this.pos.y >= h - 1)
        this.pos.x -= (this.pos.x - this.pre.x + this.acc.x);
};
*/
  }
  
  const draw = () = {
/*
Point.prototype.draw = function(ctx, size) {

    if(this.fixed) {

        ctx.fillStyle = 'rgba(255,255,255,0.2)';
        ctx.beginPath();
        ctx.arc(this.pos.x, this.pos.y, size * 3, 0, two_PI, false);
        ctx.fill();
    }

    ctx.fillStyle = (this.fixed) ? '#EDEA26' : '#aaa';
    ctx.beginPath();
    ctx.arc(this.pos.x, this.pos.y, size, 0, two_PI, false);
    ctx.fill();
};
*/
  }
  
  return ({
    move,
    add_force,
    update,
    check_walls,
    draw,
  })
}

const Constraint = (in_p1, in_p2) => {
  const p1 = in_p1
  const p2 = in_p2
  const init_len = p1.pos.distance(p2.pos)
  const stretch = len * 0.15
  
  const resolve = () => {
    const dists = p2.pos.sub(p1.pos)
    const len = dists.len()
    const diff = len - init_len
    const f = dists.normalize().mul_n(diff * 0.5)
    p1.move(f)
    p2.move(f.mul_n(-1))
  }
  
  const draw = () => {
/*
Constraint.prototype.draw = function(ctx, stress) {

    if(stress) {

        var diff = this.length - this.p1.pos.distance(this.p2.pos);

        var per = Math.round(Math.min(Math.abs(diff / this.stretch), 1) * 255);

        ctx.strokeStyle = 'rgba(255, '+(255 - per)+', '+(255 - per)+', 0.8)';

    } else ctx.strokeStyle = 'rgba(255,255,255,0.8)';

    ctx.beginPath();
    ctx.moveTo(this.p1.pos.x, this.p1.pos.y);
    ctx.lineTo(this.p2.pos.x, this.p2.pos.y);
    ctx.stroke();
};
*/
  }
  
  return ({
    resolve,
    draw,
  })
}
