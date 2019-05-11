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
  fixed = in_fixed || false,
  drag_to = false
  
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
  release = () => fixed = false,
  is_fixed = () => fixed,
  set_drag = xy => drag_to = xy,
  get_drag = () => drag_to
  
  return ({
    get_pos,
    move,
    add_force,
    update,
    check_walls,
    fix,
    release,
    is_fixed,
    set_drag,
    get_drag,
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

/** XXX (0.1 + 0.2 - 0.3) == 0 in javascript is false!!! */
const f_cut = n => parseFloat(n.toFixed(5))
const f_is0 = n => (0 == f_cut(n))
const f_eq = (a, b) => f_is0(a - b)
const f_nlt = (a, b) => f_eq(a, b) || a > b // not less than >=
const f_ngt = (a, b) => f_eq(a, b) || a < b // not greater than <=

const calc_aabb = (xy_arr, margin) => {
  margin = margin || 0
  const
  last_index = xy_arr.length - 1,
  { x, y } = xy_arr[last_index]
  let
  left = x,
  right = x,
  top = y,
  bottom = y,
  i = last_index
  while (i--) {
    const { x: a, y: b } = xy_arr[i]
    if (a < left) {
      left = a
    }
    if (right < a) {
      right = a
    }
    if (b < top) {
      top = b
    }
    if (bottom < b) {
      bottom = b
    }
  }
  left -= margin
  right += margin
  top -= margin
  bottom += margin
  
  const has = ({ x, y }) => (
    f_nlt(x, left) && f_ngt(x, right) &&
    f_nlt(y, top) && f_ngt(y, bottom)
  )
  
  return ({
    left,
    right,
    top,
    bottom,
    has,
    over: ({ left, right, top, bottom }) => (
      has({ x: left, y: top }) ||
      has({ x: left, y: bottom }) ||
      has({ x: right, y: bottom }) ||
      has({ x: left, y: top })
    ),
  })
}

const is_between = (t, a, b) => (
  (f_ngt(t, a) && f_nlt(t, b)) ||
  (f_nlt(t, a) && f_ngt(t, b))
)

const create_line = (in_xy1, in_xy2) => {
  /** a x + b y + c = 0 */
  let
  a = 1,
  b = 1,
  c = 0
  const
  xy1 = in_xy1,
  xy2 = in_xy2,
  d_xy = xy1.sub(xy2)
  
  if (f_is0(d_xy.x)) {
    if (f_is0(d_xy.y)) {
      return ({
        cross: xy => f_eq(xy.x, xy1.x) && f_eq(xy.y, xy1.y),
        expand: _ => [[xy1, xy2], [xy1, xy2], [xy1, xy1], [xy2, xy2]],
      })
    } else {
      b = 0
      c = -xy1.x
    }
  } else {
    if (f_is0(d_xy.y)) {
      a = 0
      c = -xy1.y
    } else {
      a = -(d_xy.y / d_xy.x)
      c = -(a * xy1.x + xy1.y)
    }
  }
  
  const expand_line = offset => {
    const
    top_left = xy1 - offset,
    top_right = xy1 + offset,
    bot_left = xy2 - offset,
    bot_right = xy2 + offset
    return ([
      [top_left, top_right],
      [top_left, bot_left],
      [bot_left, bot_right],
      [top_right, bot_right],
    ])
  }
  
  return ({
    cross: xy => {
      const is_valid = is_between(xy.x, xy1.x, xy2.x)
      if (false == is_valid) {
        return false
      }
      if (0 == b) {
        return (
          f_ngt(xy.y, xy1.y) ||
          f_ngt(xy.y, xy2.y)
        )
      }
      if (0 == a) {
        return is_between(xy.y, xy1.y, xy2.y)
      }
      return f_ngt(a * xy.x + b * xy.y + c, 0)
    },
    expand: width => {
      const len = width / 2
      if (0 == b) {
        return expand_line(XY(0, len))
      }
      if (0 == a) {
        return expand_line(XY(len, 0))
      }
      const offset = XY(1, -a / b).normalize().mul_n(len)
      return expand_line(offset)
    },
  })
}

const polygon_has = (polygon, xy) => {
  const cross_count = 0
  for (const [ xy1, xy2 ] of polygon) {
    const line = create_line(xy1, xy2)
    if (line.cross(xy)) {
      ++cross_count
    }
  }
  return 1 == (cross_count % 2)
}

/**
The relation between line slope (ls) and perpendicular slope (ps): ps * ls = -1

x1, y1, s1,
x2, y2, s2,
x, y,

s2 = -1 / s1,
(x - x1) * s1 = y - y1, s1 * x - s1 * x1 = y - y1,
(x - x2) * s2 = y - y2, s2 * x - s2 * x2 = y - y2,
x = ((y2 - y1) - (s2 * x2 - s1 * x1)) / (s1 - s2),
*/

const serialize = ({ points, constraints }) => {
  let ps = '\n'
  for (const p of points) {
    const { x, y } = p.get_pos()
    ps += `[${x}, ${y}, ${p.is_fixed()}],\n`
  }
  let cs = '\n'
  for (const c of constraints) {
    cs += `[${points.indexOf(c.p1)}, ${points.indexOf(c.p2)}],\n`
  }
  return `{ ps: [${ps}], cs: [${cs}], }`
}

const deserialize = ({ ps, cs }) => {
  const points = []
  const constraints = []
  for (const [x, y, fixed] of ps) {
    points.push(Point(x, y, fixed))
  }
  for (const [p1, p2] of cs) {
    constraints.push(Constraint(points[p1], points[p2]))
  }
  return ({
    points,
    constraints,
  })
}
