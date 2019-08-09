let
auxiliary_lines = false

const
{
  start_player,
} = require('./player.js'),
{
  create_line,
  polygon_has,
  polygon2renderable,
  XY,
} = require('./geometry.js'),
active_clz = 'active',

setup = (be, ie) => {
  return ({
    basic: btn => {
      console.log('Editor: basic')
      ie.onclick = () => {
        const { x, y } = ie.get_mouse()
        be.create_point(x, y)
      }
    },
    mark_fix: btn => {
      console.log('Editor: mark fix')
      ie.onmousemove = () => {
        const
        { x, y } = ie.get_mouse(),
        p = be.get_point_around(x, y)
        if (p) {
          auxiliary_lines = polygon2renderable(p.expand(5))
        } else {
          auxiliary_lines = false
        }
      }
      ie.onclick = () => {
        const
        { x, y } = ie.get_mouse(),
        p = be.get_point_around(x, y)
        if (p) {
          if (p.is_fixed()) {
            p.release()
          } else {
            p.fix()
          }
        }
      }
    },
    move_fix: btn => {
      console.log('Editor: move fix')
      let
      xy_origin = false,
      unity = false,
      aabb = false
      
      const
      clear_state = () => {
        xy_origin = false
        unity = false
        aabb = false
        auxiliary_lines = false
      }
      
      ie.onmousedown = () => {
        const
        { x, y } = ie.get_mouse(),
        p = be.get_point_around(x, y)
        if (p && p.is_fixed()) {
          unity = be.get_unity_has_p(p)
          xy_origin = XY(x, y)
          aabb = unity.get_aabb()
        } else {
          clear_state()
        }
      }
      ie.onmousemove = () => {
        if (unity) {
          const
          { x, y } = ie.get_mouse(),
          xy_movement = XY(x, y).sub(xy_origin)
          
          auxiliary_lines = polygon2renderable(aabb.to_polygon(xy_movement))
        }
      }
      ie.onmouseup = () => {
        if (unity) {
          const
          { x, y } = ie.get_mouse(),
          { points } = unity,
          movement = XY(x, y).sub(xy_origin)
          
          for (const p of points) {
            p.teleport(movement)
          }
        }
        
        clear_state()
      }
    },
    mark_edge: btn => {
      console.log('Editor: mark edge')
      ie.onmousemove = () => {
        const
        { x, y } = ie.get_mouse(),
        bone = be.get_bone_around(x, y)
        if (bone) {
          auxiliary_lines = polygon2renderable(create_line(bone.p1.get_pos(), bone.p2.get_pos()).expand(5))
        } else {
          auxiliary_lines = false
        }
      }
      ie.onclick = () => {
        const
        { x, y } = ie.get_mouse(),
        bone = be.get_bone_around(x, y)
        bone && bone.set_edge(false === bone.is_edge())
      }
    },
    make_line: btn => {
      console.log('Editor: make line')
    },
    make_square: btn => {
      console.log('Editor: make square')
    },
    make_cycle: btn => {
      console.log('Editor: make cycle')
    },
    make_cloth: btn => {
      console.log('Editor: make cloth')
    },
    make_rope: btn => {
      console.log('Editor: make rope')
    },
    make_stave: btn => {
      console.log('Editor: make stave')
    },
    make_breakable: btn => {
      console.log('Editor: make breakable')
    },
    exit_editor: btn => {
      console.log('Exit editor, player take over events')
      start_player(be, ie)
      remove_class(btn, active_clz)
      
      console.log(be.to_string())
    },
  })
},
add_class = (ele, clz) => {
  ele.className += ` ${clz}`
},
remove_class = (ele, clz) => {
  const old = ele.className
  ele.className = old.replace(clz, '').trim()
},
get_auxiliary_lines = () => auxiliary_lines,
create_editor = (be, ie) => {
  /**
  be: bone engine
  ie: input engine
  */
  let
  state = false
  
  const
  config = setup(be, ie)
  
  for (const entry in config) {
    const
    selector = `#e_${entry}`,
    button = document.querySelector(selector)
    
    if (undefined == button) {
      continue
    }
    
    button.onclick = () => {
      const
      old = document.querySelector(`li.${active_clz}`)
      
      if (old) {
        remove_class(old, active_clz)
      }
      add_class(button, active_clz)
      
      auxiliary_lines = false
      ie.offall = true
      be.set_kinematics(false)
      be.clear_creating()
      config[entry](button)
    }
  }
  
  return ({
    get_auxiliary_lines,
  })
}

module.exports = {
  create_editor,
}