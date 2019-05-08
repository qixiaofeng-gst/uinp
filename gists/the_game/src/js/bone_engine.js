/*PUT geometry.js */

const BoneEngine = (in_canvas, in_options) => {
  let
  mouse = XY(),
  mouse_point = false,
  draw_points = []

  const
  canvas = in_canvas,
  options = in_options,
  { width, height } = canvas,
  
  constraints = [],
  points = [],

  gravity = options.gravity || XY(0, 0.98),
  point_size = options.point_size || 2,
  show_stress = options.show_stress || false,
  
  key = {
    ctrl: false,
    alt: false,
  },
  get_mouse_point = () => {
    let closest = false
    let i = points.length;
    while(i--) {
      const point = points[i]
      if(point.get_pos().distance(mouse) < 10) {
          closest = point
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
    if (key.ctrl) {
      return
    }

    iter = iter || 6

    const delta = 1 / iter
    
    let n = iter
    while(n--) {
      if (mouse_point) {
        const p = mouse.sub(mouse_point.get_pos()).div_n(iter)
        mouse_point.move(p)
      }

      for(const p of points) {
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
  }
  
  canvas.oncontextmenu = e => e.preventDefault()

  canvas.onmousedown = e => {
    mouse = XY(e.offsetX, e.offsetY)
    mouse.down = true
    
    let p = get_mouse_point()
    
    if (e.which == 3) {
      if (p) {
        remove_point(p)
      }
    } else {
      if (key.ctrl) {
        if (false == p) {
          p = Point(mouse.x, mouse.y, key.alt)
          points.push(p)
        }

        if (draw_points.length) {
          constraints.push(Constraint(p, draw_points[draw_points.length - 1]))
        }
        draw_points.push(p)
      } else {
        mouse_point = p
      }
    }
  }

  canvas.onmouseup = e => {
    mouse_point = false
  }

  canvas.onmousemove = e => {
    mouse = XY(e.offsetX, e.offsetY)
  }
  
  document.onkeydown = e => {
    if (e.keyCode == 17) {
      key.ctrl = true
    } else if (e.keyCode == 16) {
      draw_points = []
    } else if (e.keyCode == 18) {
      key.alt = true
    }
  }

  document.onkeyup = e => {
      if (e.keyCode == 17) {
          key.ctrl = false
          draw_points = []
      } else if (e.keyCode == 18) {
          key.alt = false
      }
  }
  
  return ({
    add_point,
    add_constraint,
    add_shape,
    get_lines,
    update,
  })
}
