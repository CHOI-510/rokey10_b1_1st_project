import { BufferGeometry, Float32BufferAttribute, Mesh, type Object3D } from "three";

// Vertex clustering is used only for the CPU SVG fallback, never for control or WebGL assets.
export function simplifyForSvg(root: Object3D) {
  root.traverse(object=>{
    if (!(object instanceof Mesh)) return;
    const source: BufferGeometry=object.geometry;
    const positions=source.getAttribute("position");
    if (!positions || positions.count<600) return;
    source.computeBoundingBox();
    const bounds=source.boundingBox!;
    const cell=Math.max(bounds.max.x-bounds.min.x,bounds.max.y-bounds.min.y,bounds.max.z-bounds.min.z)/32;
    if (!(cell>0)) return;
    const bins=new Map<string,number>(), sums:number[][]=[], mapping:number[]=[];
    for(let i=0;i<positions.count;i++){
      const x=positions.getX(i),y=positions.getY(i),z=positions.getZ(i);
      const key=`${Math.round(x/cell)},${Math.round(y/cell)},${Math.round(z/cell)}`;
      let bin=bins.get(key);
      if(bin===undefined){bin=sums.length;bins.set(key,bin);sums.push([0,0,0,0]);}
      const sum=sums[bin];sum[0]+=x;sum[1]+=y;sum[2]+=z;sum[3]++;mapping.push(bin);
    }
    const materialIndices=new Map<number,number[]>(),seen=new Set<string>();
    const count=source.index?.count??positions.count;
    for(let i=0;i<count;i+=3){
      const ids=[0,1,2].map(k=>mapping[source.index?source.index.getX(i+k):i+k]);
      if(new Set(ids).size<3)continue;
      const material=source.groups.find(group=>i>=group.start&&i<group.start+group.count)?.materialIndex??0;
      const key=`${material}:${[...ids].sort((a,b)=>a-b).join(",")}`;
      if(seen.has(key))continue;seen.add(key);
      const indices=materialIndices.get(material)||[];indices.push(...ids);materialIndices.set(material,indices);
    }
    const geometry=new BufferGeometry();
    geometry.setAttribute("position",new Float32BufferAttribute(sums.flatMap(sum=>[sum[0]/sum[3],sum[1]/sum[3],sum[2]/sum[3]]),3));
    const indices:number[]=[];
    for(const [material,group] of materialIndices){geometry.addGroup(indices.length,group.length,material);indices.push(...group);}
    geometry.setIndex(indices);geometry.computeVertexNormals();geometry.computeBoundingSphere();
    object.geometry=geometry;source.dispose();
  });
}
