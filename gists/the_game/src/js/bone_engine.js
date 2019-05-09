/*PUT geometry.js */

const BoneEngine = (in_canvas) => {
  let draw_points = false

  const
  canvas = in_canvas,
  
  { width, height } = canvas,
  constraints = [],
  points = [],

  gravity = XY(0, 0.98),
  
  get_point_around = (x, y, range) => {
    let closest = false
    const target = XY(x, y)
    for (const p of points) {
      if(p.get_pos().distance(target) < range) {
          closest = p
      }
    }
    return closest
  },
  remove_point = point => {
    let i = constraints.length
    while (i--) {
      const constraint = constraints[i]
      if (constraint.p1 == point || constraint.p2 == point) {
        constraints.splice(constraints.indexOf(constraint), 1)
      }
    }

    if (points.includes(point)) {
      points.splice(points.indexOf(point), 1)
    }
  },
  add_point = (x, y, fixed) => {
    const point = Point(x, y, fixed)
    points.push(point)
    return point
  },
  add_constraint = (p1, p2) => {
    const c = Constraint(p1, p2)
    constraints.push(c)
    return c
  },
  add_shape = ({ points: ps, constraints: cs }) => {
    if (Array.isArray(ps)) {
      points.splice(points.length, 0, ...ps)
    }
    if (Array.isArray(cs)) {
      constraints.splice(constraints.length, 0, ...cs)
    }
  },
  update = iter => {
    if (draw_points) {
      return
    }

    iter = iter || 6

    const delta = 1 / iter
    
    let n = iter
    while(n--) {
      for(const p of points) {
        const drag = p.get_drag()
        if (drag) {
          const s = drag.sub(p.get_pos()).div_n(iter)
          p.move(s)
        }
        
        p.add_force(gravity)
        p.update(delta)
        p.check_walls(0, 0, width, height)
      }

      for(const c of constraints) {
        c.resolve()
      }
    }
  },
  get_lines = () => {
    const lines = []
    for (const c of constraints) {
      const { x: x1, y: y1 } = c.p1.get_pos()
      const { x: x2, y: y2 } = c.p2.get_pos()
      lines.splice(lines.length, 0, x1, y1, x2, y2)
    }
    return lines
  },
  get_points = () => {
    const ps = []
    for (const p of points) {
      const { x, y } = p.get_pos()
      const size = p.is_fixed() ? 10 : 6
      ps.push([
        x, y, size,
      ])
    }
    return ps
  },
  
  is_constraint_exists = (pa, pb) => {
    if (pa == pb) {
      return true
    }
    for (const { p1, p2 } of constraints) {
      const exists = p1 == pa && p2 == pb
      const exists_r = p1 == pb && p2 == pa
      if (exists || exists_r) {
        return true
      }
    }
    return false
  },
  start_editing = () => {
    draw_points = []
  },
  draw_point = p => {
    if (false == draw_points) {
      return
    }
    
    const { x, y } = p.get_pos()
    const exists = get_point_around(x, y)
    if (exists) {
      draw_points.push(exists)
      const { length } = draw_points
      if (1 == length) {
        draw_points.push(exists)
        return
      }
      const prev = draw_points[length - 2]
      if (false == is_constraint_exists(exists, prev)) {
        constraints.push(Constraint(exists, prev))
      }
      return
    }
    
    draw_points.push(p)
    const { length } = draw_points
    if (1 == length) {
      return
    }
    const prev = draw_points[length - 2]
    if (2 == length) {
      points.push(prev)
    }
    points.push(p)
    constraints.push(Constraint(p, prev))
  },
  end_editing = () => {
    draw_points = false
  }
  
  return ({
    add_point,
    add_constraint,
    add_shape,
    get_lines,
    get_points,
    update,
    start_editing,
    draw_point,
    end_editing,
  })
}
