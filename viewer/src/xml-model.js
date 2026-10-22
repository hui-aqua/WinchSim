export function xmlToViewerResult(xmlText, name) {
  const parser = new DOMParser();
  const xml = parser.parseFromString(xmlText, 'application/xml');
  const err = xml.querySelector('parsererror');
  if (err) throw Error('XML error: ' + (err.textContent || 'syntax error'));
  const nodes = [];
  const nodeMap = new Map();
  xml.querySelectorAll('Nodes > node, model > Nodes > node, node').forEach(n => {
    const id = n.getAttribute('id');
    const x = parseFloat(n.getAttribute('x') || '0');
    const y = parseFloat(n.getAttribute('y') || '0');
    const z = parseFloat(n.getAttribute('z') || '0');
    if (id !== null && !isNaN(x) && !isNaN(y) && !isNaN(z)) {
      const nid = 'n-' + id;
      nodeMap.set(id, [x, y, z]);
      nodes.push({
        id: nid,
        position: [x, y, z],
        role: n.getAttribute('role') || (/winch/i.test(n.getAttribute('name') || '') ? 'winch' : /roller/i.test(n.getAttribute('name') || '') ? 'roller' : /joint/i.test(n.getAttribute('name') || '') ? 'joint' : 'structural'),
        label: n.getAttribute('name') || id
      });
    }
  });
  const elements = [];
  const components = [];
  const componentColors = new Map();
  let elemCounter = 1;
  xml.querySelectorAll('Components > beam, Components > truss, beam, truss').forEach(comp => {
    const tag = comp.tagName.toLowerCase();
    const compId = comp.getAttribute('id') || comp.getAttribute('name') || tag;
    const compName = comp.getAttribute('name') || compId;
    const category = tag === 'beam' ? 'beams' : 'trusses';
    const colorEl = comp.querySelector('color');
    const sectionEl = comp.querySelector('wizard');
    const diameterMm = parseFloat(sectionEl?.getAttribute('outerDiameter') || '');
    const area = parseFloat(comp.querySelector('mooring')?.getAttribute('areal') || '');
    const sectionRadius = Number.isFinite(diameterMm) ? diameterMm / 2000 : Number.isFinite(area) && area > 0 ? Math.sqrt(area / Math.PI) : undefined;
    let color = undefined;
    if (colorEl) {
      const r = parseFloat(colorEl.getAttribute('red') || '0.5');
      const g = parseFloat(colorEl.getAttribute('green') || '0.5');
      const b = parseFloat(colorEl.getAttribute('blue') || '0.5');
      color = '#' + [r, g, b].map(v => Math.round(Math.max(0, Math.min(1, v)) * 255).toString(16).padStart(2, '0')).join('');
    }
    components.push({ id: compId, label: compName, category, color, section_radius_m: sectionRadius });
    componentColors.set(compId, color);
    comp.querySelectorAll('element').forEach(el => {
      const elId = el.getAttribute('id') || String(elemCounter++);
      const sId = el.getAttribute('StartNode_ID') || el.getAttribute('startNodeId');
      const eId = el.getAttribute('EndNode_ID') || el.getAttribute('endNodeId');
      if (sId && eId && nodeMap.has(sId) && nodeMap.has(eId)) {
        elements.push({
          id: elId,
          component_id: compId,
          category: category,
          role: el.getAttribute('role') || (tag === 'beam' ? 'beam' : 'rope'),
          nodes: ['n-' + sId, 'n-' + eId],
          type: 'segment',
          name: el.getAttribute('name') || elId,
          color: color,
          section_radius_m: sectionRadius,
          component_name: compName
        });
      }
    });
  });
  elements.forEach(element => { if (!element.color) element.color = componentColors.get(element.component_id); });
  const envEl = xml.querySelector('environment');
  const waterline = envEl ? parseFloat(envEl.getAttribute('waterline_z_m') || '0') : 0;
  const nameAttr = xml.querySelector('model')?.getAttribute('name') || name;
  const descAttr = xml.querySelector('model')?.getAttribute('description') || 'Static model geometry';
  return {
    project: { name: nameAttr, description: descAttr },
    model_status: 'model_input_preview',
    scene: {
      schema: 'winchsim.scene.v1',
      coordinate_system: { handedness: 'right', up: 'z', units: 'm' },
      waterline_z: waterline,
      nodes: nodes,
      elements: elements,
      components: components,
      primitives: []
    },
    metrics: [],
    results_count: 1,
    results: [{ time_s: 0.0 }]
  };
}
