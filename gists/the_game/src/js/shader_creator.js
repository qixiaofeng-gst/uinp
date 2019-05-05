const gl = document.getElementById('canvas').getContext('webgl')
const oes_tex_float = gl.getExtension('OES_texture_float')
const oes_tex_float_linear = gl.getExtension('OES_texture_float_linear')
if (null == oes_tex_float || null == oes_tex_float_linear) {
  console.warn('Missing extension.')
} else {
  console.log('Extensions ready:', oes_tex_float, oes_tex_float_linear)
}
const create_shader = (type, src) => {
  const shader = gl.createShader(type)
  gl.shaderSource(shader, src)
  gl.compileShader(shader)
  if (gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
    return shader
  } else {
    console.error(
      `Failed to compile shader, shader type:${type}`,
      gl.getShaderInfoLog(shader)
    )
    gl.deleteShader(shader)
    return null
  }
}
const location_config = {
  attribute: (program, name) => gl.getAttribLocation(program, name),
  uniform: (program, name) => gl.getUniformLocation(program, name),
}
const data_type_config = {
  vec2: (desc, val) => gl.uniform2fv(desc.location, val),
  sampler2D: (desc, val) => {
    gl.activeTexture(gl.TEXTURE0 + desc.index)
    gl.bindTexture(gl.TEXTURE_2D, val)
    gl.uniform1i(desc.location, desc.index)
  },
  float: (desc, val) => gl.uniform1f(desc.location, val),
}
const parse_variables = src => {
  const reg_var = /(attribute|uniform)\s+\w+\s+\w+;$/gm
  const var_strs = src.match(reg_var)
  const vars = {}
  const indexer = {}
  if (null === var_strs) {
    return vars
  }
  for (const v of var_strs) {
    const a3 = v.trim().split(/\s+/)
    const [var_type, data_type, name_str] = a3
    const name = name_str.substr(0, name_str.length - 1)
    const index = indexer[data_type] || 0
    if (vars.hasOwnProperty(name)) {
      console.warn(`Duplicated variable name "${name}" detected`)
    }
    vars[name] = {
      var_type,
      data_type,
      index,
    }
    indexer[data_type] = index + 1
  }
  return vars
}
const parse_locations = (vars, program) => {
  for (const k in vars) {
    const v = vars[k]
    v.location = location_config[v.var_type](program, k)
    v.program = program
  }
}
const create_vertex_shader = src => {
  return create_shader(gl.VERTEX_SHADER, src)
}
const create_fragment_shader = src => {
  return create_shader(gl.FRAGMENT_SHADER, src)
}
const get_src = id => {
  return document.getElementById(id).innerText
}
const create_shader_program = (vs_id, fs_id) => {
  const vs_src = get_src(vs_id)
  const fs_src = get_src(fs_id)
  const vars = parse_variables(vs_src + fs_src)
  const vs = create_vertex_shader(vs_src)
  const fs = create_fragment_shader(fs_src)
  const program = gl.createProgram()
  gl.attachShader(program, vs)
  gl.attachShader(program, fs)
  gl.linkProgram(program)
  if (gl.getProgramParameter(program, gl.LINK_STATUS)) {
    parse_locations(vars, program)
    program.vars = vars
    return new Proxy({
      use: () => gl.useProgram(program)
    }, {
      get: (_, key, __) => {
        if (vars.hasOwnProperty(key)) {
          return vars[key]
        }
        return _[key]
      },
      set: (_, key, value, __) => {
        const desc = vars[key]
        data_type_config[desc.data_type](desc, value)
        return value
      }
    })
  } else {
    console.error(
      `Failed to link program`,
      gl.getProgramInfoLog(program)
    )
    gl.deleteProgram(program)
    return null
  }
}