const
{
  XY,
} = require('./geometry.js'),
pass_per_update = 6,
delta_per_update = 1 / pass_per_update,
gravity = XY(0, 0.98),
start_player = (be, ie) => {
  let dragging = false
  const
  update = (points, bones, pass) => {
    if (undefined == pass) {
      pass = 0
    }
    
    for(const p of points) {
      const drag = p.get_drag()
      if (drag) {
        const s = drag.sub(p.get_pos()).mul_n(delta_per_update)
        p.move(s)
      }
      
      p.add_force(gravity)
      p.update(delta_per_update)
      const { width, height } = be.get_size()
      p.check_walls(0, 0, width, height)
    }

    for(const c of bones) {
      c.resolve()
    }
    
    if (pass < pass_per_update) {
      update(points, bones, pass + 1)
    }
  },
  end_drag = () => {
    if (dragging) {
      dragging.set_drag(false)
    }
    dragging = false
  },
  start_drag = () => {
    const { x, y } = ie.get_mouse()
    dragging = be.get_point_around(x, y)
    if (dragging) {
      dragging.set_drag(XY(x, y))
    }
  },
  do_drag = () => {
    if (dragging) {
      const { x, y } = ie.get_mouse()
      dragging.set_drag(XY(x, y))
    }
  }
  
  ie.onmousedown = start_drag
  ie.onmousemove = do_drag
  ie.onmouseup = end_drag
  
  ie.onclick = () => {
    const { x, y } = ie.get_mouse()
    
    const us = be.get_unities()
    for (const { points, bones } of us) {
      const xy_arr = []
      for (const p of points) {
        xy_arr.push(p.get_pos())
      }
      if (calc_aabb(xy_arr, 10).has({ x, y })) {
        for (const { p1, p2 } of bones) {
          const line_area = create_line(p1.get_pos(), p2.get_pos()).expand(5)
          if (polygon_has(line_area, { x, y })) {
            return
          }
        }
      }
    }
    
    if (ie.is_ctrl_down()) {
      be.create_point(x, y, ie.is_alt_down())
    }
  }
  
  be.set_kinematics({
    update,
  })
}

module.exports = {
  start_player,
}