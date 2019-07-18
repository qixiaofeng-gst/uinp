const
start_player = (be, ie) => {
  let dragging = false
  const
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
  
  ie.onkeydown = () => {
    if (ie.ctrl == ie.get_code()) {
      be.start_editing()
    }
  }
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
  ie.onkeyup = () => {
    if (ie.ctrl == ie.get_code()) {
      be.end_editing()
      be.detect_unities()
    }
  }
}

module.exports = {
  start_player,
}