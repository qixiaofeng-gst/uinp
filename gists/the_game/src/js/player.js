const
{
  XY,
} = require('./geometry.js'),
pass_per_update = 1,
delta_per_update = 1 / pass_per_update,
gravity = XY(0, 0.098),
start_player = (be, ie) => {
  let dragging = false
  const
  collide = () => {
    const
    unities = be.get_unities()
    for (let i = 0; i < unities.length; ++i) {
      for (let j = 0; j < unities.length; ++j) {
        if (i === j) {
          continue
        }
        
        const
        a = unities[i],
        b = unities[j]
        if (a.collide(b)) {
          //TODO console.log('damn the colliding')
        }
      }
    }
  },
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
    } else {
      collide()
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
  
  be.set_kinematics({
    update,
  })
}

module.exports = {
  start_player,
}