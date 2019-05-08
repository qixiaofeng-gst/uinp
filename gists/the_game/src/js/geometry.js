const XY = (v1, v2) => {
  const
  x = v1 || 0,
  y = v2 || 0,
  cal_len = (a, b) => Math.sqrt(a * a + b * b),
  len = cal_len(x, y),
  
  sub = ({ x: a, y: b }) => XY(x - a, y - b),
  add = ({ x: a, y: b }) => XY(x + a, y + b),
  mul = ({ x: a, y: b }) => XY(x * a, y * b),
  div = ({ x: a, y: b }) => XY(x / a, y / b),
  mul_n = n => XY(x * n, y * n),
  div_n = n => XY(x / n, y / n),
  
  distance = ({ x: a, y: b }) => cal_len(x - a, y - b),
  normalize = () => ((len > 0) ? XY(x / len, y / len) : XY())
  
  return ({
    x, y, len,
    sub, add, mul, div,
    mul_n, div_n,
    normalize,
    distance,
  })
}

const Point = (x, y, in_fixed) => {
  let
  pos = XY(x, y),
  pre = XY(x, y),
  acc = XY(),
  fixed = in_fixed || false
  
  const
  move = v => {
    if (fixed) {
      return
    }
    pos = pos.add(v)
  },
  add_force = v => {
    if (fixed) {
      return
    }
    acc = acc.add(v)
  },
  update = delta => {
    if (fixed) {
      return
    }
    
    const old_pos = pos
    
    delta *= delta
    acc = acc.mul_n(delta)
    pos = pos.add(pos.sub(pre).add(acc))
    acc = XY()
    
    pre = old_pos
  },
  check_walls = (in_x, in_y, in_w, in_h) => {
    let x = Math.max(in_x + 1, Math.min(in_w - 1, pos.x))
    let y = Math.max(in_y + 1, Math.min(in_h - 1, pos.y))
    if (y >= (in_h - 1)) {
      x -= (pos.x - pre.x + acc.x)
    }
    pos = XY(x, y)
  },
  get_pos = () => pos,
  fix = () => fixed = true,
  is_fixed = () => fixed
  
  return ({
    get_pos,
    move,
    add_force,
    update,
    check_walls,
    fix,
    is_fixed,
  })
}

const Constraint = (in_p1, in_p2) => {
  const
  p1 = in_p1,
  p2 = in_p2,
  init_len = p1.get_pos().distance(p2.get_pos()),
  stretch = init_len * 0.15,
  
  resolve = () => {
    const
    dists = p2.get_pos().sub(p1.get_pos()),
    len = dists.len,
    diff = len - init_len,
    f = dists.normalize().mul_n(diff * 0.5)
    
    p1.move(f)
    p2.move(f.mul_n(-1))
  }
  
  return ({
    p1, p2,
    resolve,
  })
}

const Rectangle = (x, y, w, h) => {
  const
  p1 = Point(x, y),
  p2 = Point(x + w, y),
  p3 = Point(x + w, y + h),
  p4 = Point(x, y + h),

  c1 = Constraint(p1, p2),
  c2 = Constraint(p2, p3),
  c3 = Constraint(p3, p4),
  c4 = Constraint(p4, p1),
  c5 = Constraint(p1, p3)

  points = [p1, p2, p3, p4],
  constraints = [c1, c2, c3, c4, c5],
  fix = idx => points[idx % points.length].fix()
  
  return ({
    points,
    constraints,
    fix,
  })
}

const serialize = ({ points, constraints }) => {
  //TODO minimize the output string
  /**
  ps: [
    [1, 2, false],
    [3, 4, true],
    ...
  ],
  cs: [
    [0, 1],
    [2, 3],
    ...
  ]
  */
}
