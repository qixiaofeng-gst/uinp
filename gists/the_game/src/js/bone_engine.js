/*PUT geometry.js */

const BoneEngine = (in_canvas) => {
  let draw_points = false

  const
  canvas = in_canvas,
  
  { width, height } = canvas,
  bones = [],
  points = [],
  unities = [],

  gravity = XY(0, 0.98),
  
  get_point_around = (x, y, range) => {
    range = range || 9
    let closest = false
    const target = XY(x, y)
    for (const p of points) {
      if(p.get_pos().distance(target) < range) {
          closest = p
      }
    }
    return closest
  },
  batch_add = ({ points: ps, bones: cs }) => {
    if (Array.isArray(ps)) {
      points.splice(points.length, 0, ...ps)
    }
    if (Array.isArray(cs)) {
      bones.splice(bones.length, 0, ...cs)
    }
  },
  
  get_relevant_bones = p => {
    const cs = []
    for (const c of bones) {
      if (c.is_unity_parsed) {
        continue
      }
      const is_p1 = (c.p1 == p)
      if (is_p1 || c.p2 == p) {
        cs.push(c)
        c.is_unity_parsed = true
        const subs = get_relevant_bones(is_p1 ? c.p2 : c.p1)
        cs.splice(cs.length, 0, ...subs)
      }
    }
    return cs
  },
  mark_unity_parsed = (ps, p) => {
    p.is_unity_parsed = true
    if (false == ps.includes(p)) {
      ps.push(p)
    }
  },
  detect_unities = () => {
    for (const p of points) {
      p.is_unity_parsed = false
    }
    for (const c of bones) {
      c.is_unity_parsed = false
    }
    unities.splice(0, unities.length)
    for (const p of points) {
      if (p.is_unity_parsed) {
        continue
      }
      const ps = []
      const cs = get_relevant_bones(p)
      for (const c of cs) {
        mark_unity_parsed(ps, c.p1)
        mark_unity_parsed(ps, c.p2)
      }
      unities.push({
        points: ps,
        bones: cs,
      })
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

      for(const c of bones) {
        c.resolve()
      }
    }
  },
  
  get_lines = () => {
    const lines = []
    for (const c of bones) {
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
  get_unities = () => unities,
  to_string = () => serialize({
    points, bones
  }),
  
  is_constraint_exists = (pa, pb) => {
    if (pa == pb) {
      return true
    }
    for (const { p1, p2 } of bones) {
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
  remove_point = point => {
    let i = bones.length
    while (i--) {
      const constraint = bones[i]
      if (constraint.p1 == point || constraint.p2 == point) {
        bones.splice(bones.indexOf(constraint), 1)
      }
    }

    if (points.includes(point)) {
      points.splice(points.indexOf(point), 1)
    }
  },
  create_point = (x, y, fixed) => {
    if (false === draw_points) {
      return
    }
    
    const exists = get_point_around(x, y)
    if (exists) {
      if (fixed) {
        exists.fix()
      } else {
        exists.release()
      }
      draw_points.push(exists)
      const { length } = draw_points
      if (1 == length) {
        draw_points.push(exists)
        return
      }
      const prev = draw_points[length - 2]
      if (false == is_constraint_exists(exists, prev)) {
        bones.push(Bone(exists, prev))
      }
      return
    }
    
    const p = Point(x, y, fixed)
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
    bones.push(Bone(p, prev))
  },
  end_editing = () => {
    draw_points = false
  }
  
  return ({
    batch_add,
    
    get_lines,
    get_points,
    get_point_around,
    get_unities,
    to_string,
    
    update,
    detect_unities,
    start_editing,
    create_point,
    end_editing,
  })
}
