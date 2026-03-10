// === 1. 初始化场景、相机和渲染器 ===
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x222222);

const camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.1, 100);
// 注意：因为后续我们把根节点设为Z-up，相机的视角也略作调整
camera.position.set(0.5, -1.5, 1.5); 
camera.up.set(0, 0, 1); // 告诉相机Z轴是向上

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

const controls = new THREE.OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 0.2);
controls.update();

// === 2. 光照和网格 ===
scene.add(new THREE.AmbientLight(0x555555));
const dirLight = new THREE.DirectionalLight(0xffffff, 1.2);
dirLight.position.set(2, -2, 3);
scene.add(dirLight);

// 将网格旋转90度，匹配Z-up坐标系
const gridHelper = new THREE.GridHelper(2, 20, 0x444444, 0x888888);
gridHelper.rotation.x = Math.PI / 2;
scene.add(gridHelper);

// 建立机器人根节点，模拟 MuJoCo Z轴朝上的世界系
const robotRoot = new THREE.Group();
scene.add(robotRoot);

// 画一个简易的躯干 (Torso)
const torsoMesh = new THREE.Mesh(
    new THREE.BoxGeometry(0.15, 0.1, 0.4),
    new THREE.MeshPhongMaterial({ color: 0x333333, transparent: true, opacity: 0.8 })
);
torsoMesh.position.set(0, 0, 0.2); // 在Z-up坐标系下稍微抬高
robotRoot.add(torsoMesh);


// === 3. G1 左臂与右臂真实运动学参数 (提取自 g1_mocap_29dof.xml) ===
// 注意：Three.js的四元数是 [x, y, z, w]，而MuJoCo是 [w, x, y, z]，此处已做了转换调换。
const leftArmParams = [
    { name: "shoulder_pitch", pos: [0.0039563, 0.10022, 0.24778], quat: [0.139201, 1.38722e-05, -9.86868e-05, 0.990264], axis: [0, 1, 0], range: [-3.0892, 1.149] },
    { name: "shoulder_roll", pos: [0, 0.038, -0.013831], quat: [-0.139172, 0, 0, 0.990268], axis: [1, 0, 0], range: [-0.6, 2.2515] },
    { name: "shoulder_yaw", pos: [0, 0.00624, -0.1032], quat: [0, 0, 0, 1], axis: [0, 0, 1], range: [-1.4, 2.0] },
    { name: "elbow", pos: [0.015783, 0, -0.080518], quat: [0, 0, 0, 1], axis: [0, 1, 0], range: [-1.0472, 1.7] },
    { name: "wrist_roll", pos: [0.1, 0.00188791, -0.01], quat: [0, 0, 0, 1], axis: [1, 0, 0], range: [-1.97222, 1.97222] },
    { name: "wrist_pitch", pos: [0.038, 0, 0], quat: [0, 0, 0, 1], axis: [0, 1, 0], range: [-1.61443, 1.61443] },
    { name: "wrist_yaw", pos: [0.046, 0, 0], quat: [0, 0, 0, 1], axis: [0, 0, 1], range: [-1.61443, 1.61443] }
];

const rightArmParams = [
    { name: "shoulder_pitch", pos: [0.0039563, -0.10021, 0.24778], quat: [-0.139201, 1.38722e-05, 9.86868e-05, 0.990264], axis: [0, 1, 0], range: [-3.0892, 1.149] },
    { name: "shoulder_roll", pos: [0, -0.038, -0.013831], quat: [0.139172, 0, 0, 0.990268], axis: [1, 0, 0], range: [-2.2515, 0.6] },
    { name: "shoulder_yaw", pos: [0, -0.00624, -0.1032], quat: [0, 0, 0, 1], axis: [0, 0, 1], range: [-2.0, 1.4] },
    { name: "elbow", pos: [0.015783, 0, -0.080518], quat: [0, 0, 0, 1], axis: [0, 1, 0], range: [-1.0472, 1.7] },
    { name: "wrist_roll", pos: [0.1, -0.00188791, -0.01], quat: [0, 0, 0, 1], axis: [1, 0, 0], range: [-1.97222, 1.97222] },
    { name: "wrist_pitch", pos: [0.038, 0, 0], quat: [0, 0, 0, 1], axis: [0, 1, 0], range: [-1.61443, 1.61443] },
    { name: "wrist_yaw", pos: [0.046, 0, 0], quat: [0, 0, 0, 1], axis: [0, 0, 1], range: [-1.61443, 1.61443] }
];


// === 4. 构建真实 1-DoF 运动链 ===
function createArm(isLeft, params) {
    let currentParent = robotRoot; 
    let jointObjects = []; 

    for(let i = 0; i < params.length; i++) {
        const p = params[i];
        
        // 1. 建立固定的局部坐标系 (处理 Offset 偏移和 Quat 初始姿态)
        const localFrame = new THREE.Group();
        localFrame.position.set(p.pos[0], p.pos[1], p.pos[2]);
        localFrame.quaternion.set(p.quat[0], p.quat[1], p.quat[2], p.quat[3]);
        currentParent.add(localFrame);

        // 2. 绘制 1-DoF 旋转轴实体 (用细长圆柱代表轴承方向)
        const axisColor = isLeft ? 0x00aaff : 0xff4444;
        const axisMesh = new THREE.Mesh(
            new THREE.CylinderGeometry(0.012, 0.012, 0.05, 16),
            new THREE.MeshPhongMaterial({ color: axisColor })
        );
        // Three.js 的圆柱默认沿 Y 轴，根据 XML 的 axis 让它对齐正确的物理旋转轴
        if(p.axis[0] === 1 || p.axis[0] === -1) axisMesh.rotation.z = Math.PI / 2; // X轴
        if(p.axis[2] === 1 || p.axis[2] === -1) axisMesh.rotation.x = Math.PI / 2; // Z轴
        localFrame.add(axisMesh);

        // 3. 建立专门用于接收角度输入的旋转层 (Rotator)
        const rotator = new THREE.Group();
        localFrame.add(rotator);
        
        // 存储该组，包含 params 信息，方便UI更新
        jointObjects.push({ rotator: rotator, param: p });

        // 4. 绘制可视化的“连杆” (指向下一个关节)
        if (i < params.length - 1) {
            const nextP = params[i+1];
            // 计算连杆长度 (欧几里得距离)
            const dist = Math.hypot(nextP.pos[0], nextP.pos[1], nextP.pos[2]);
            const linkMesh = new THREE.Mesh(
                new THREE.CylinderGeometry(0.008, 0.008, 1, 8),
                new THREE.MeshPhongMaterial({ color: 0x888888 })
            );
            
            // 计算指向目标节点的向量
            const dir = new THREE.Vector3(nextP.pos[0], nextP.pos[1], nextP.pos[2]);
            const linkWrapper = new THREE.Group();
            
            // 对齐连杆
            const defaultDir = new THREE.Vector3(0, 1, 0);
            const quat = new THREE.Quaternion().setFromUnitVectors(defaultDir, dir.clone().normalize());
            linkWrapper.quaternion.copy(quat);
            
            linkMesh.position.y = dist / 2;
            linkMesh.scale.set(1, dist, 1);
            
            linkWrapper.add(linkMesh);
            rotator.add(linkWrapper);
        }

        currentParent = rotator;
    }
    return jointObjects;
}

const leftJoints = createArm(true, leftArmParams);
const rightJoints = createArm(false, rightArmParams);


// === 5. UI 生成与绑定逻辑 (读取真实 Limit 限位) ===
function createUI(containerId, joints, prefix) {
    const container = document.getElementById(containerId);
    
    joints.forEach((jointObj, index) => {
        const p = jointObj.param;
        const group = document.createElement('div');
        group.className = 'slider-group';

        const label = document.createElement('label');
        label.innerHTML = `<span>[${index}] ${p.name}</span> <span id="${prefix}-val-${index}" class="val-display">0.00</span>`;
        
        const slider = document.createElement('input');
        slider.type = 'range';
        // 绑定真实的上下限范围
        slider.min = p.range[0].toString();
        slider.max = p.range[1].toString();
        slider.step = '0.01';
        slider.value = '0';

        // 监听滑动事件
        slider.addEventListener('input', (e) => {
            const val = parseFloat(e.target.value);
            document.getElementById(`${prefix}-val-${index}`).innerText = val.toFixed(2);
            
            // 应用 1-DoF 旋转: setRotationFromAxisAngle 可以精准按向量转动
            const axisVec = new THREE.Vector3(p.axis[0], p.axis[1], p.axis[2]);
            jointObj.rotator.setRotationFromAxisAngle(axisVec, val);
        });

        group.appendChild(label);
        group.appendChild(slider);
        container.appendChild(group);
    });
}

createUI('left-sliders', leftJoints, 'left');
createUI('right-sliders', rightJoints, 'right');


// === 6. 渲染循环 ===
function animate() {
    requestAnimationFrame(animate);
    renderer.render(scene, camera);
}
animate();

window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});

// === 7. WebSocket 通信逻辑 ===
// 连接到 Ubuntu 上的 WebSocket 服务 (端口设为 8765)
const ws = new WebSocket('ws://192.168.3.38:8765');

ws.onopen = () => {
    console.log("Connected to Python Bridge!");
    // 给 UI 加个绿色边框表示连接成功
    document.getElementById('left-ui').style.borderColor = '#00ffcc';
    document.getElementById('right-ui').style.borderColor = '#00ffcc';
};

// 采集当前 14 个关节的数据并发送
function sendJointData() {
    if (ws.readyState !== WebSocket.OPEN) return;

    // 提取左臂和右臂当前的角度
    const leftAngles = leftJoints.map(j => j.rotator.rotation[j.param.axis.indexOf(1) !== -1 ? ['x','y','z'][j.param.axis.indexOf(1)] : ['x','y','z'][j.param.axis.indexOf(-1)]] * j.param.axis.find(v => v !== 0));
    // 简单点，直接利用我们存在 UI 里的数值：
    const leftData = [];
    const rightData = [];
    for(let i=0; i<7; i++) {
        leftData.push(parseFloat(document.getElementById(`left-val-${i}`).innerText));
        rightData.push(parseFloat(document.getElementById(`right-val-${i}`).innerText));
    }

    const payload = {
        left_arm: leftData,
        right_arm: rightData
    };
    
    ws.send(JSON.stringify(payload));
}

// 修改原有的 createUI，在滑动时触发发送
function createUI_with_ws(containerId, joints, prefix) {
    const container = document.getElementById(containerId);
    joints.forEach((jointObj, index) => {
        const p = jointObj.param;
        const group = document.createElement('div');
        group.className = 'slider-group';

        const label = document.createElement('label');
        label.innerHTML = `<span>[${index}] ${p.name}</span> <span id="${prefix}-val-${index}" class="val-display">0.00</span>`;
        
        const slider = document.createElement('input');
        slider.type = 'range';
        slider.min = p.range[0].toString();
        slider.max = p.range[1].toString();
        slider.step = '0.01';
        slider.value = '0';

        slider.addEventListener('input', (e) => {
            const val = parseFloat(e.target.value);
            document.getElementById(`${prefix}-val-${index}`).innerText = val.toFixed(2);
            
            const axisVec = new THREE.Vector3(p.axis[0], p.axis[1], p.axis[2]);
            jointObj.rotator.setRotationFromAxisAngle(axisVec, val);

            // 【新增】每次滑动都发送数据给后端
            //sendJointData(); 
        });

        group.appendChild(label);
        group.appendChild(slider);
        container.appendChild(group);
    });
}

// 替换之前的 createUI 调用
document.getElementById('left-sliders').innerHTML = '';
document.getElementById('right-sliders').innerHTML = '';
createUI_with_ws('left-sliders', leftJoints, 'left');
createUI_with_ws('right-sliders', rightJoints, 'right');


// === 8. 持续高频发送 (解决 1 秒超时回弹问题) ===
// 设置为每 33 毫秒发送一次 (约 30Hz)
setInterval(() => {
    // 只有在 WebSocket 处于连接状态时才发送
    if (ws && ws.readyState === WebSocket.OPEN) {
        sendJointData();
    }
}, 33);