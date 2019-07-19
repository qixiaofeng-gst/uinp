const {
  XY,
  Point,
  Bone,
} = require('./geometry.js')

const BoneEngine = (in_canvas) => {
  let
  kinematics = false,
  creating = false

  const
  { width, height } = in_canvas,
  bones = [],
  points = [],
  unities = [],
  
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
  
  update = () => {
    if (kinematics && kinematics.update) {
      kinematics.update(points, bones)
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
    const exists = get_point_around(x, y)
    if (exists) {
      if (creating) {
        if (creating.is_exist) {
          if (false == is_constraint_exists(exists, creating.p)) {
            bones.push(Bone(exists, creating.p))
          }
          creating = false
        } else {
          points.push(creating.p)
          bones.push(Bone(exists, creating.p))
          creating = false
        }
      } else {
        creating = {
          is_exist: true,
          p: exists,
        }
      }
    } else {
      const p = Point(x, y, fixed)
      if (creating) {
        if (false == creating.is_exist) {
          points.push(creating.p)
        }
        points.push(p)
        bones.push(Bone(creating.p, p))
        creating = false
      } else {
        creating = {
          is_exist: false,
          p,
        } 
      }
    }
  },
  clear_creating = () => {
    creating = false
    detect_unities()
  },
  set_kinematics = obj => {
    kinematics = obj
  },
  get_size = () => ({ width, height })
  
  return ({
    batch_add,
    
    get_lines,
    get_points,
    get_point_around,
    get_unities,
    to_string,
    
    update,
    detect_unities,
    create_point,
    clear_creating,
    
    set_kinematics,
    get_size,
  })
}

module.exports = BoneEngine