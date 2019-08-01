let
auxiliary_lines = false

const
{
  start_player,
} = require('./player.js'),
active_clz = 'active',
/**
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
*/
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
                auxiliary_lines = []
                for (const [ v1, v2 ] of line_area) {
                  auxiliary_lines.push(v1.x)
                  auxiliary_lines.push(v1.y)
                  auxiliary_lines.push(v2.x)
                  auxiliary_lines.push(v2.y)
                }
                return
              }
            }
          }
        }
        auxiliary_lines = false
      }
    },
    move_fix: btn => {
      console.log('Editor: move fix')
    },
    mark_edge: btn => {
      console.log('Editor: mark edge')
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