const
{
  XY,
  Point,
  Bone,
  Unity,
  min_bone_len,
  polygon_has,
  create_line,
  serialize,
} = require('./geometry.js')

const
BoneEngine = (in_canvas) => {
  let
  kinematics = false,
  creating = false

  const
  { width, height } = in_canvas,
  bones = [],
  points = [],
  unities = [],
  default_radius = 9,
  
  get_default_radius = () => default_radius,
  get_point_around = (x, y, range) => {
    range = range || default_radius
    let closest = false
    const target = XY(x, y)
    for (const p of points) {
      if(p.get_pos().distance(target) < range) {
          closest = p
      }
    }
    return closest
  },
  get_bone_around = (x, y, range) => {
    range = range || default_radius
    for (const unity of unities) {
      if (unity.get_aabb(range).has({ x, y })) {
        for (const bone of unity.bones) {
          const
          { p1, p2 } = bone,
          line = create_line(p1.get_pos(), p2.get_pos()),
          line_area = line.expand(range)
          if (polygon_has(line_area, { x, y })) {
            return bone
          }
        }
      }
    }
    return false
  },
  get_unity_has_p = p => {
    if (p) {
      for (const unity of unities) {
        const { points } = unity
        for (const point of points ) {
          if (point == p) {
            return unity
          }
        }
      }
    }
    return false
  },
  batch_add = ({ points: ps, bones: cs }) => {
    if (Array.isArray(ps)) {
      points.splice(points.length, 0, ...ps)
    }
    if (Array.isArray(cs)) {
      bones.splice(bones.length, 0, ...cs)
    }
    
    detect_unities()
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
      unities.push(Unity(ps, cs))
    }
  },
  
  update = () => {
    if (kinematics && kinematics.update) {
      kinematics.update(points, bones)
    }
  },
  
  get_bones = () => bones,
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
  
  is_bone_exists = (pa, pb) => {
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
  create_point = (x, y) => {
    const exists = get_point_around(x, y)
    if (exists) {
      if (creating) {
        if (creating.p === exists) {
          return
        }
        if (creating.is_exist) {
          if (false == is_bone_exists(exists, creating.p)) {
            bones.push(Bone(exists, creating.p))
          }
        } else {
          points.push(creating.p)
          bones.push(Bone(exists, creating.p))
        }
        
      }
      creating = {
        is_exist: true,
        p: exists,
      }
    } else {
      const p = Point(x, y)
      if (creating) {
        if (creating.p.get_pos().sub(p.get_pos()).len < min_bone_len) {
          return
        }
        if (false == creating.is_exist) {
          points.push(creating.p)
        }
        points.push(p)
        bones.push(Bone(creating.p, p))
        creating = {
          is_exist: true,
          p,
        }
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
    
    get_bones,
    get_points,
    get_point_around,
    get_bone_around,
    get_default_radius,
    get_unities,
    get_unity_has_p,
    to_string,
    
    update,
    create_point,
    clear_creating,
    
    set_kinematics,
    get_size,
  })
}

module.exports = BoneEngine