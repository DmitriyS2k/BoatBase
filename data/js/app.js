'use strict';

let state = {};
let ws = null;
let wsOk = false;
const MODES = ['MANUAL','SAVE_WP','AUTO'];
const WP_NAMES = ['🏠 Дом','📍 Точка 1','📍 Точка 2','📍 Точка 3'];

// ── WebSocket ───────────────────────────────────────────────────
function connectWS() {
  ws = new WebSocket('ws://' + location.host + '/ws');
  ws.onopen  = () => { wsOk=true;  setWS(true); };
  ws.onclose = () => { wsOk=false; setWS(false); setTimeout(connectWS,2000); };
  ws.onerror = () => { ws.close(); };
  ws.onmessage = (e) => {
    try { state = JSON.parse(e.data); updateInPlace(); } catch(ex){}
  };
}
function setWS(ok) {
  const el = document.getElementById('wsStatus');
  if (!el) return;
  el.textContent = ok ? 'WS ✓' : 'WS ✗';
  el.className = 'badge ' + (ok ? 'ok' : 'bad');
}

// ── Tab routing ─────────────────────────────────────────────────
let curPage = 'dash';
document.querySelectorAll('.tab').forEach(t => {
  t.onclick = () => {
    document.querySelectorAll('.tab').forEach(x=>x.classList.remove('active'));
    t.classList.add('active');
    const newPage = t.dataset.page;
    if (newPage !== curPage) { curPage = newPage; buildPage(); }
  };
});

// ── buildPage: строит DOM один раз при смене вкладки ────────────
function buildPage() {
  const pg = document.getElementById('page');
  if (!pg) return;
  switch(curPage) {
    case 'dash':     pg.innerHTML = tmplDash();     break;
    case 'ota':      pg.innerHTML = tmplOta();      break;
    case 'joystick': pg.innerHTML = tmplJoystick(); initJoystick(); break;
    case 'nav':      pg.innerHTML = tmplNav();      break;
    case 'log':      pg.innerHTML = tmplLog();      loadLog(); loadNavLogStatus(); break;
    case 'settings': pg.innerHTML = tmplSettings(); loadSettings(); break;
    case 'help':     pg.innerHTML = tmplHelp();     break;
    case 'debug':    pg.innerHTML = tmplDebug();    break;
  }
  updateInPlace();
}

// ── updateInPlace: обновляет только данные, не трогает input ────
function setText(id, val) { const e=document.getElementById(id); if(e) e.textContent=val; }

function updateInPlace() {
  const s = state;
  const rc = document.getElementById('rcStatus');
  if (rc) { rc.textContent=s.rc?'RC ✓':'RC ✗'; rc.className='badge '+(s.rc?'ok':'bad'); }

  if (curPage==='dash') {
    setText('d-sat',   s.sat ?? '—');
    setText('d-hdop',  s.hdop ?? '—');
    // Три состояния GPS
    const fixEl = document.getElementById('d-fix');
    if (fixEl) {
      if (s.fix)                         { fixEl.textContent='✅ Фикс есть';    fixEl.style.color='#22c55e'; }
      else if (s.gpsActive)              { fixEl.textContent='🔍 Поиск...';     fixEl.style.color='#f59e0b'; }
      else                               { fixEl.textContent='❌ Нет модуля';   fixEl.style.color='#ef4444'; }
    }
    setText('d-lat',   s.lat ?? '—');
    setText('d-lon',   s.lon ?? '—');
    setText('d-speed', (s.speed ?? 0) + ' км/ч');
    if (s.motorTemp != null) {
      const t = parseFloat(s.motorTemp);
      const el = document.getElementById('d-temp');
      if (el) {
        el.textContent = t < -50 ? 'нет датчика' : t.toFixed(1) + ' °C';
        el.style.color = t >= 60 ? '#ef4444' : t >= 50 ? '#f97316' : '';
      }
    }
    setText('d-heading', (s.heading ?? 0) + '°');
    const modeStr = MODES[s.mode] || 'MANUAL';
    const mb = document.getElementById('d-mode');
    if (mb) { mb.textContent=modeStr; mb.className='mode-badge mode-'+modeStr; }
    setText('d-ml', s.ml ?? 1500);
    setText('d-mr', s.mr ?? 1500);
    const dr = document.getElementById('d-dist-row');
    if (dr) dr.style.display = s.nav ? '' : 'none';
    setText('d-dist', (s.dist ?? 0) + ' м');
    // Лимит скорости: показываем как процент от максимума
    if (s.speedLim != null) {
      const pct = Math.round((s.speedLim - 1500) / (s.speedLim > 1500 ? 4 : 1));
      const p = Math.round((s.speedLim - 1500) * 100 / 400);
      setText('d-spd', s.speedLim + ' PWM (' + Math.max(0,Math.min(100,p)) + '%)');
    }
    drawCompass();
  }

  if (curPage==='nav') {
    const WP_SHORT = ['Дом','WP1','WP2','WP3'];
    // Статус навигации
    const navEl = document.getElementById('n-nav');
    if (navEl) {
      if (s.nav) {
        navEl.textContent = '🟢 Едем → ' + (WP_SHORT[s.wpTarget] ?? 'WP'+s.wpTarget);
        navEl.style.color = '#22c55e';
      } else {
        navEl.textContent = '⬜ Стоп';
        navEl.style.color = '#64748b';
      }
    }
    setText('n-dist',    s.nav  ? (s.dist ?? 0) + ' м' : '—');
    setText('n-hdg',     (s.targetHdg ?? 0) + '°');
    // Расстояние до выбранной точки (ch5) — всегда показываем
    const selDistEl = document.getElementById('n-dist-sel');
    if (selDistEl) {
      const wp = (s.wp || [])[s.wpSel || 0];
      if (wp && wp.v) {
        selDistEl.textContent = (s.distSel ?? 0) + ' м';
      } else {
        selDistEl.textContent = 'точка не задана';
      }
    }
    const fill = document.getElementById('distFill');
    if (fill) fill.style.width = Math.min(100,(s.dist||0)/5)+'%';
    const wp = s.wp || [], sel = s.wpSel ?? 0;
    for(let i=0;i<4;i++){
      const box=document.getElementById('wpbox-'+i); if(!box) continue;
      const w=wp[i]||{};
      // selected = выбрана ch5, target = к ней едем
      const isSelected = i===sel;
      const isTarget   = s.nav && i===s.wpTarget;
      box.className=['wp-box', w.v?'valid':'', isSelected?'selected':'', isTarget?'target':''].join(' ').trim();
      const coords=box.querySelector('.wp-coords');
      if(coords) coords.textContent=w.v ? w.lat+'\n'+w.lon : 'не задана';
      const st=box.querySelector('.wp-status');
      if(st) st.textContent = isTarget ? '🚤' : (w.v?'✅':'⬜');
    }
    drawCompass();
  }

  if (curPage==='debug') {
    const chs=s.chs||[];
    for(let i=1;i<=10;i++){
      setText('ch-'+i, chs[i-1]??'—');
      const bar=document.getElementById('chbar-'+i);
      if(bar){ const pct=((chs[i-1]||1500)-1000)/10; bar.style.width=Math.max(0,Math.min(100,pct))+'%'; }
    }
    const modeStr=MODES[s.mode]||'MANUAL';
    setText('dbg-mode', modeStr);
    setText('dbg-wpsel', s.wpSel??0);
    // ch5 индикатор диапазона
    const v5 = chs[4]||1500;
    const zone = v5<1250?'0 (Дом)':v5<1500?'1':v5<1750?'2':'3';
    setText('dbg-wp-zone', 'ch5='+v5+' → точка '+zone);
  }

  if (curPage==='settings') {
    // Живые показания компаса для подбора оси
    setText('liveHdg', (s.heading ?? 0) + '°');
    setText('liveX',   s.rawX ?? '—');
    setText('liveY',   s.rawY ?? '—');
    const calibBtn = document.getElementById('calibBtn');
    const calibPrg = document.getElementById('calibProgress');
    if (s.calib) {
      if (calibBtn) { calibBtn.textContent='⏳ Калибровка... '+(s.calibPct||0)+'%'; calibBtn.disabled=true; }
      if (calibPrg) { calibPrg.style.display=''; calibPrg.value=s.calibPct||0; }
    } else {
      if (calibBtn) { calibBtn.textContent='🧭 Начать калибровку'; calibBtn.disabled=false; }
      if (calibPrg) calibPrg.style.display='none';
    }
  }
}

// ── Шаблоны ─────────────────────────────────────────────────────
function tmplDash() {
  return `
<div class="card">
  <h2>GPS</h2>
  <div class="row"><span>Спутников</span><span id="d-sat">—</span></div>
  <div class="row"><span>HDOP</span><span id="d-hdop">—</span></div>
  <div class="row"><span>Фикс</span><span id="d-fix">—</span></div>
  <div class="row"><span>Широта</span><span id="d-lat">—</span></div>
  <div class="row"><span>Долгота</span><span id="d-lon">—</span></div>
  <div class="row"><span>Скорость</span><span id="d-speed">—</span></div>
  <button onclick="gpsReset()" style="margin-top:8px;background:#ef4444">Сбросить GPS (холодный старт)</button>
</div>
<div class="card">
  <h2>Мотор</h2>
  <div class="row"><span>Температура</span><span id="d-temp">—</span></div>
</div>
<div class="card">
  <h2>Компас</h2>
  <div class="compass-wrap"><canvas id="compass" width="130" height="130"></canvas></div>
  <div class="big" id="d-heading">0°</div>
</div>
<div class="card">
  <h2>Режим</h2>
  <div style="text-align:center;padding:6px"><span id="d-mode" class="mode-badge mode-MANUAL">MANUAL</span></div>
  <div class="row"><span>Левый ESC</span><span id="d-ml">1500</span></div>
  <div class="row"><span>Правый ESC</span><span id="d-mr">1500</span></div>
  <div class="row" id="d-dist-row" style="display:none"><span>До точки</span><span id="d-dist">0 м</span></div>
  <div class="row" id="d-spd-row"><span>Лимит скорости (ch6)</span><span id="d-spd">—</span></div>
</div>`;
}

function tmplNav() {
  let boxes='';
  for(let i=0;i<4;i++){
    boxes+=`<div class="wp-box" id="wpbox-${i}">
      <div class="wp-name">${WP_NAMES[i]}</div>
      <div class="wp-coords">не задана</div>
      <div class="wp-status">⬜</div>
    </div>`;
  }
  return `
<div class="card">
  <h2>Точки маршрута</h2>
  <div class="wp-grid">${boxes}</div>
  <p style="font-size:.75rem;color:#64748b;margin-top:6px">Выбор точки — крутилка ch5 на пульте (4 зоны)</p>
</div>
<div class="card">
  <h2>Навигация</h2>
  <div class="compass-wrap"><canvas id="compass" width="130" height="130"></canvas></div>
  <div class="row"><span>Статус</span><span id="n-nav">⬜ Стоп</span></div>
  <div class="row"><span>До цели</span><span id="n-dist">—</span></div>
  <div class="row"><span>Целевой курс</span><span id="n-hdg">0°</span></div>
  <div class="row"><span>До выбранной (ch5)</span><span id="n-dist-sel">—</span></div>
  <div class="dist-bar"><div class="dist-fill" id="distFill" style="width:0%"></div></div>
  <p style="font-size:.8rem;color:#64748b;margin-top:8px">
    ⚠️ AUTO: ch9→верх. Стик вперёд (ch2≥1900) = к точке. Стик назад (ch2≤1100) = домой.
  </p>
</div>`;
}

function tmplHelp() {
  return `
<div class="card">
  <h2>Управление с пульта</h2>
  <div class="help-row"><b>ch9</b><span>Режим: низ=MANUAL, середина=SAVE WP, верх=AUTO</span></div>
  <div class="help-row"><b>ch5</b><span>Выбор точки (крутилка): 1000-1249=Дом, 1250-1499=Точка1, 1500-1749=Точка2, 1750-2000=Точка3</span></div>
  <div class="help-row"><b>ch6</b><span>Лимит скорости в AUTO (крутилка): от 50% до 100% от максимальной скорости</span></div>
  <div class="help-row"><b>ch1/ch2</b><span>Прямое управление в режиме MANUAL (tank mix на пульте: правый стик X+Y)</span></div>
  <div class="help-row"><b>ch3</b><span>Круиз-контроль: если >1750 — включается удержание курса по компасу</span></div>
  <div class="help-row"><b>ch4 в круизе</b><span>Подруливание: центр = держит курс, отклонение = ручной поворот. После отпускания фиксирует новый курс</span></div>
  <div class="help-row"><b>ch4 в SAVE WP</b><span>Управление WiFi: стик вправо >1800 = включить, стик влево <1200 = выключить</span></div>
  <div class="help-row"><b>ch2 ≥1600</b><span>В режиме AUTO — старт движения к выбранной точке (ch5)</span></div>
  <div class="help-row"><b>ch2 ≤1400</b><span>В режиме AUTO — старт возврата домой (точка 0)</span></div>
  <div class="help-row"><b>ch1 ≥1600</b><span>В режиме SAVE WP — сохранить текущую GPS позицию как выбранную точку</span></div>
</div>

<div class="card">
  <h2>Телеметрия на пульте (OpenI6X)</h2>
  <p class="help-intro">Все сенсоры ID 0x0002, добавляй вручную если не нашлись через Discover.</p>
  <div class="help-row"><b>inst1 RPM</b><span>Режим: 0=MANUAL, 1=SAVE WP, 2=AUTO</span></div>
  <div class="help-row"><b>inst2 RPM</b><span>Выбранная точка: 0=Дом, 1-3=Точки</span></div>
  <div class="help-row"><b>inst3 RPM</b><span>Количество спутников GPS</span></div>
  <div class="help-row"><b>inst4 RPM</b><span>Дистанция до точки (м): в AUTO — до цели, иначе — до дома</span></div>
</div>

<div class="card">
  <h2>PID автопилот — что крутить</h2>
  <p class="help-intro">PID управляет тем как корабль исправляет отклонение от курса. Настраивай на воде, начинай с Kp. Стартовые значения проверены на воде с моторами 550.</p>
  <div class="help-param">
    <div class="help-pname">Kp (пропорциональный) — старт 3.0</div>
    <div class="help-pdesc">Основная сила поворота. <b>Слишком мало</b> — корабль идёт дугой, медленно выходит на курс. <b>Слишком много</b> — рыскает влево-вправо, мелкие рывки. Для моторов 550 рекомендуется 3.0-4.5.</div>
  </div>
  <div class="help-param">
    <div class="help-pname">Ki (интегральный) — старт 1.5</div>
    <div class="help-pdesc">Убирает постоянное смещение от курса (ветер, течение, разброс моторов). <b>Слишком много</b> — медленные нарастающие колебания. При настройке сначала выставь Ki=0, настрой Kp, потом добавляй Ki от 0 по 0.5.</div>
  </div>
  <div class="help-param">
    <div class="help-pname">Kd (дифференциальный) — старт 0.5</div>
    <div class="help-pdesc">Гасит колебания и рывки. <b>Слишком много</b> — дёргается, резко реагирует на шум компаса. Если есть мелкие рывки при движении — увеличь до 1.0-2.0. Обычно не трогают после первичной настройки.</div>
  </div>
  <div class="help-param">
    <div class="help-pname">Нелинейность выхода (pidCurve) — старт 1.0</div>
    <div class="help-pdesc">Смягчает реакцию на мелкие ошибки курса, не теряя силу на больших. Моторы 550 слишком мощные — при линейном PID (curve=1.0) даже пара градусов шума компаса даёт заметный рывок мотора, корабль дёргается влево-вправо. <b>Увеличь до 1.5-2.5</b> — у центра (маленькие ошибки) коррекция станет заметно слабее, а у краёв (реальные большие отклонения) останется почти такой же сильной. Пробуй это в первую очередь если есть мелкие рывки при в целом верном курсе.</div>
  </div>
  <div class="help-param">
    <div class="help-pname">Макс. разница моторов (maxDiff) — старт 150</div>
    <div class="help-pdesc">Ограничивает насколько сильно автопилот может отклонить один мотор от другого. <b>Мало (50-100)</b> — плавные повороты, широкая дуга захода на точку, меньше рыскания. <b>Много (250-350)</b> — крутые повороты, заход на точку прямее. Начни с 150, увеличивай если идёт слишком широкой дугой.</div>
  </div>
  <div class="help-param">
    <div class="help-pname">Период коррекции курса (navInterval) — старт 200мс</div>
    <div class="help-pdesc">Как часто автопилот пересчитывает и корректирует курс. <b>100мс</b> — часто, быстрая реакция но больше рывков. <b>300-500мс</b> — реже, плавнее, меньше рывков но медленнее реагирует на отклонение. Если рывки есть даже при малом maxDiff — увеличь до 300-400мс.</div>
  </div>
  <div class="help-param">
    <div class="help-pname">Сглаживание курса на точку (bearingAlpha) — старт 0.15</div>
    <div class="help-pdesc">Фильтр GPS шума при расчёте направления на точку. GPS координаты прыгают на 1-3м — это вызывает мелкие рывки моторов. <b>0.05</b> — сильное сглаживание, плавно но медленно реагирует на изменение направления. <b>0.15</b> — хороший баланс. <b>0.5-1.0</b> — слабый фильтр, быстрая реакция но больше рывков. Уменьши если есть рывки при прямом движении к точке.</div>
  </div>
  <div class="help-param">
    <div class="help-pname">Скорость автопилота</div>
    <div class="help-pdesc">PWM тяга в режиме AUTO (1500=стоп, 1900=полный газ). Стартуй с 1620-1650. Крутилка ch6 на пульте задаёт от 50% до 100% этого значения. Слишком высокая скорость — промахивается мимо точки из-за инерции.</div>
  </div>
  <div class="help-param">
    <div class="help-pname">Замедление перед точкой</div>
    <div class="help-pdesc"><b>С какого расстояния</b> — за сколько метров начинать тормозить. <b>Скорость торможения</b> — до какого PWM замедляться (1530-1580). Если проскакивает точку — увеличь расстояние или снизь скорость торможения.</div>
  </div>
  <div class="help-param">
    <div class="help-pname">Радиус прибытия</div>
    <div class="help-pdesc">За сколько метров до точки считать «доехали». GPS даёт точность ~3-5м, меньше этого ставить бессмысленно. Рекомендуется 3-5м.</div>
  </div>
</div>

<div class="card">
  <h2>Круиз-контроль — что крутить</h2>
  <div class="help-param">
    <div class="help-pname">Коэффициент круиза (cruiseGain) — старт 0.8</div>
    <div class="help-pdesc">Сила удержания курса в ручном круиз-режиме (ch3>1750). <b>Мало</b> — медленно возвращается на курс после волны. <b>Много</b> — дёргается при коррекции. 0.5-1.5 обычно хорошо для моторов 550.</div>
  </div>
</div>

<div class="card">
  <h2>Trim моторов</h2>
  <p class="help-intro">Если на прямом газу корабль уходит в сторону — это разброс ESC или винтов. Компенсируется trimом.</p>
  <div class="help-param">
    <div class="help-pname">Как настроить</div>
    <div class="help-pdesc">Выйди на воду, дай ~70% газа прямо без руля в MANUAL режиме. Уходит вправо — увеличь trimLeft или уменьши trimRight. Уходит влево — наоборот. Trim применяется только при активном газе и улучшает работу автопилота.</div>
  </div>
</div>

<div class="card">
  <h2>Рекомендуемый порядок настройки на воде</h2>
  <ol class="help-steps">
    <li>Откалибруй компас вдали от металла и моторов (Настройки → Компас). Выбери правильную ось.</li>
    <li>Установи магнитное склонение для своего региона (magnetic-declination.com). Для Украины ~+7..+9°.</li>
    <li>Настрой trim моторов в MANUAL — поплыви прямо и убери уход в сторону.</li>
    <li>Проверь круиз (ch3>1750) — держит курс, ch4 подруливает, после отпускания фиксирует новый курс.</li>
    <li>Сохрани точку Дом: выйди на воду, переключи SAVE WP, ch1≥1600 при хорошем GPS (≥6 спутников).</li>
    <li>Установи стартовые значения PID: Kp=3.0, Ki=1.5, Kd=0.5, maxDiff=150, bearingAlpha=0.15.</li>
    <li>Дай команду вернуться домой (AUTO → ch2≤1400). Смотри как едет.</li>
    <li>Рыскает влево-вправо рывками → сначала увеличь pidCurve до 1.5-2.5 (мощные моторы 550 иначе дёргаются от шума). Если не помогло — уменьши Kp или maxDiff. Идёт широкой дугой → увеличь maxDiff.</li>
    <li>Мелкие рывки при прямом движении → уменьши bearingAlpha до 0.08.</li>
    <li>Постоянно уходит в одну сторону → добавь Ki от 0 по 0.5.</li>
    <li>Промахивается мимо точки → увеличь расстояние замедления или уменьши скорость.</li>
  </ol>
</div>`;
}

function tmplSettings() {
  return `
<div class="card">
  <h2>Автопилот (AUTO режим)</h2>
  <label>Скорость в AUTO, PWM <span class="hint">1500=стоп · 1900=полный</span></label>
  <input type="number" id="cruiseSpeed" min="1500" max="1900" step="10" value="1650">

  <label style="margin-top:10px">Замедление перед точкой</label>
  <div class="two-col">
    <div>
      <label class="sub">С какого расстояния, м</label>
      <input type="number" id="slowdownDist" min="1" max="30" step="0.5" value="5">
    </div>
    <div>
      <label class="sub">Мин. скорость, PWM</label>
      <input type="number" id="slowdownSpeed" min="1500" max="1700" step="10" value="1550">
    </div>
  </div>

  <label style="margin-top:10px">Радиус прибытия, м <span class="hint">рекомендуется 3-5м</span></label>
  <input type="number" id="arrivalRadius" min="1" max="50" step="0.5" value="3">

  <label style="margin-top:10px">Мин. спутников для AUTO</label>
  <input type="number" id="minSatellites" min="3" max="20" step="1" value="5">

  <button class="btn btn-blue" onclick="saveSettings()">💾 Сохранить</button>
</div>

<div class="card">
  <h2>PID регулятор курса</h2>
  <p class="help-intro">Подробное описание — вкладка Справка. Стартовые значения проверены на воде (Katerok).</p>

  <label>Kp = <span id="kpVal">—</span> <span class="hint">основная сила поворота · вилянье → уменьши</span></label>
  <input type="range" id="pidKp" min="0.5" max="20" step="0.1" value="3.0">

  <label>Ki = <span id="kiVal">—</span> <span class="hint">убирает постоянное отклонение от курса</span></label>
  <input type="range" id="pidKi" min="0" max="20" step="0.1" value="1.5">

  <label>Kd = <span id="kdVal">—</span> <span class="hint">гасит колебания · обычно не трогать</span></label>
  <input type="range" id="pidKd" min="0" max="5" step="0.05" value="0.5">

  <label>Нелинейность (pidCurve) = <span id="pcVal">—</span> <span class="hint">1.0=линейно · выше=мягче у центра, жёстче у краёв</span></label>
  <input type="range" id="pidCurve" min="0.5" max="3" step="0.1" value="1.0">

  <label>Круиз gain = <span id="crgVal">—</span> <span class="hint">удержание курса в ручном круизе (ch3)</span></label>
  <input type="range" id="cruiseGain" min="0.1" max="5" step="0.1" value="0.8">
  <div class="row"><span>Макс. разница моторов (maxDiff)</span><span id="mdVal">150</span></div>
  <input type="range" id="maxDiff" min="10" max="400" step="5" value="150">
  <div class="row"><span>Сглаживание курса на точку (bearingAlpha)</span><span id="baVal">0.15</span></div>
  <input type="range" id="bearingAlpha" min="0.05" max="1.0" step="0.05" value="0.15">
  <div class="row"><span>Период коррекции курса (navInterval, мс)</span><span id="niVal">200</span></div>
  <input type="range" id="navInterval" min="100" max="1000" step="50" value="200">

  <button class="btn btn-blue" onclick="saveSettings()">💾 Сохранить</button>
</div>

<div class="card">
  <h2>Trim моторов</h2>
  <p class="help-intro">Корабль уходит в сторону на прямом газе? Скомпенсируй здесь. <a onclick="sendPrompt('Как настроить trim моторов?')" style="color:var(--text-accent);cursor:pointer">Подробнее →</a></p>
  <label>Левый мотор <span id="trimLVal">0</span></label>
  <input type="range" id="trimLeft" min="-200" max="200" step="1" value="0">
  <label>Правый мотор <span id="trimRVal">0</span></label>
  <input type="range" id="trimRight" min="-200" max="200" step="1" value="0">
  <div style="display:flex;gap:8px;margin-top:6px;font-size:.8rem;color:#64748b">
    <span>Левый ESC: <b id="previewL" style="color:var(--text-primary)">—</b></span>
    <span>Правый ESC: <b id="previewR" style="color:var(--text-primary)">—</b></span>
  </div>
  <button class="btn btn-blue" onclick="saveSettings()">💾 Сохранить</button>
</div>

<div class="card">
  <h2>Компас</h2>
  <label>Магнитное склонение, °</label>
  <input type="number" id="compassDecl" min="-30" max="30" step="0.5" value="0">

  <label style="margin-top:12px">Ориентация осей чипа</label>
  <p style="font-size:.78rem;color:#64748b;margin:2px 0 8px">
    Направи нос на север, смотри на показание. Меняй вариант пока не покажет ~0°.
  </p>
  <div style="display:flex;align-items:center;gap:10px;margin-bottom:8px">
    <div style="font-size:1.3rem;font-weight:500;min-width:60px" id="liveHdg">—°</div>
    <div style="font-size:.8rem;color:#64748b">сырые: X=<span id="liveX">—</span> Y=<span id="liveY">—</span></div>
  </div>
  <select id="compassAxis" style="width:100%;padding:6px;border-radius:6px;border:1px solid var(--border);background:var(--surface-1);color:var(--text-primary);font-size:.9rem">
    <option value="0">0 — стандарт математический</option>
    <option value="1">1 — X→нос, Y→вправо ✅ попробуй первым</option>
    <option value="2">2 — X→нос, Y→влево</option>
    <option value="3">3 — X→нос, чип снизу</option>
    <option value="4">4 — X→нос, повёрнут 180°</option>
    <option value="5">5 — X→корма, Y→вправо</option>
    <option value="6">6 — X→нос, Y→влево v2</option>
    <option value="7">7 — X→корма, Y→влево</option>
  </select>

  <label style="margin-top:10px">Мёртвая зона компаса ±° <span class="hint">2°=тихо 5°=волнение</span></label>
  <input type="number" id="compassDeadzone" min="0" max="15" step="0.5" value="2">

  <button class="btn btn-blue" style="margin-top:10px" onclick="saveCompass()">💾 Сохранить компас</button>
</div>

<div class="card">
  <h2>Калибровка компаса</h2>
  <p style="font-size:.85rem;color:#64748b;margin-bottom:10px">
    Сначала выбери правильный режим осей выше, потом калибруй.
    Медленно повращай корабль 2-3 оборота за 15 сек (горизонтально).
  </p>
  <button class="btn btn-orange" id="calibBtn" onclick="startCalib()">🧭 Начать калибровку</button>
  <progress id="calibProgress" max="100" value="0" style="display:none;width:100%;margin-top:10px"></progress>
</div>`;
}

function tmplDebug() {
  let rows='';
  for(let i=1;i<=10;i++){
    rows+=`<div class="ch-row">
      <span class="ch-lbl">CH${i}</span>
      <div class="ch-bar-bg"><div class="ch-bar-fill" id="chbar-${i}" style="width:50%"></div></div>
      <span class="ch-val" id="ch-${i}">—</span>
    </div>`;
  }
  return `
<div class="card">
  <h2>Каналы RC</h2>
  ${rows}
  <div class="row" style="margin-top:10px"><span>Режим</span><span id="dbg-mode">—</span></div>
  <div class="row"><span>Точка (wpSel)</span><span id="dbg-wpsel">—</span></div>
  <div class="row"><span>ch5 зона</span><span id="dbg-wp-zone">—</span></div>
</div>`;
}

// ── Компас canvas ────────────────────────────────────────────────
function drawCompass() {
  const c=document.getElementById('compass'); if(!c) return;
  const ctx=c.getContext('2d'), cx=65, cy=65, r=58;
  ctx.clearRect(0,0,130,130);
  ctx.strokeStyle='#334155'; ctx.lineWidth=2;
  ctx.beginPath(); ctx.arc(cx,cy,r,0,Math.PI*2); ctx.stroke();
  const dirs=['N','NE','E','SE','S','SW','W','NW'];
  ctx.font='bold 11px system-ui'; ctx.textAlign='center'; ctx.textBaseline='middle';
  dirs.forEach((d,i)=>{
    const a=(i*45-90)*Math.PI/180, rr=d.length===1?r-8:r-12;
    ctx.fillStyle=d==='N'?'#ef4444':'#64748b';
    ctx.fillText(d, cx+rr*Math.cos(a), cy+rr*Math.sin(a));
  });
  // Стрелка курса
  const hdg=(state.heading||0)*Math.PI/180-Math.PI/2;
  ctx.save(); ctx.translate(cx,cy); ctx.rotate(hdg);
  ctx.fillStyle='#3b82f6';
  ctx.beginPath(); ctx.moveTo(0,-r+14); ctx.lineTo(-7,0); ctx.lineTo(7,0); ctx.closePath(); ctx.fill();
  ctx.restore();
  // Целевой курс (оранжевый тик)
  if (state.nav && state.targetHdg != null) {
    const th=state.targetHdg*Math.PI/180-Math.PI/2;
    ctx.save(); ctx.translate(cx,cy); ctx.rotate(th);
    ctx.strokeStyle='#f59e0b'; ctx.lineWidth=3;
    ctx.beginPath(); ctx.moveTo(0,-(r-18)); ctx.lineTo(0,-(r-2)); ctx.stroke();
    ctx.restore();
  }
}

// ── Settings ─────────────────────────────────────────────────────
function loadSettings() {
  fetch('/api/settings')
    .then(r=>r.json())
    .then(d=>{
      const set=(id,v)=>{const e=document.getElementById(id);if(e)e.value=v;};
      set('pidKp',        d.pidKp);
      set('pidKi',        d.pidKi);
      set('pidKd',        d.pidKd);
      set('pidCurve',     d.pidCurve ?? 1.0);
      set('cruiseGain',   d.cruiseGain);
      set('maxDiff',      d.maxDiff ?? 150);
      set('bearingAlpha', d.bearingAlpha ?? 0.15);
      set('navInterval',  d.navInterval ?? 200);
      set('cruiseSpeed',  d.cruiseSpeed);
      set('slowdownDist', d.slowdownDist ?? 5);
      set('slowdownSpeed',d.slowdownSpeed ?? 1550);
      set('arrivalRadius',d.arrivalRadius);
      set('minSatellites',d.minSatellites);
      set('compassDecl',      d.compassDecl);
      set('compassAxis',      d.compassAxis ?? 1);
      set('compassDeadzone',  d.compassDeadzone ?? 2);
      set('trimLeft',     d.trimLeft ?? 0);
      set('trimRight',    d.trimRight ?? 0);
      updRangeLabels();
    }).catch(()=>{});
  ['pidKp','pidKi','pidKd','pidCurve','cruiseGain','maxDiff','bearingAlpha','navInterval','trimLeft','trimRight'].forEach(id=>{
    document.getElementById(id)?.addEventListener('input', updRangeLabels);
  });
  // Живое изменение оси — сразу сохраняем и видим результат
  document.getElementById('compassAxis')?.addEventListener('change', saveCompass);
}
function updRangeLabels() {
  const bind=(id,outId,dec)=>{
    const e=document.getElementById(id);
    if(e) document.getElementById(outId).textContent=parseFloat(e.value).toFixed(dec);
  };
  bind('pidKp','kpVal',1);
  bind('pidKi','kiVal',1);
  bind('pidKd','kdVal',2);
  bind('pidCurve','pcVal',2);
  bind('cruiseGain','crgVal',1);
  bind('maxDiff','mdVal',0);
  bind('bearingAlpha','baVal',2);
  bind('navInterval','niVal',0);

  // Trim ползунки
  const tl = parseInt(document.getElementById('trimLeft')?.value  || 0);
  const tr = parseInt(document.getElementById('trimRight')?.value || 0);
  const tlEl = document.getElementById('trimLVal');
  const trEl = document.getElementById('trimRVal');
  if (tlEl) tlEl.textContent = (tl >= 0 ? '+' : '') + tl;
  if (trEl) trEl.textContent = (tr >= 0 ? '+' : '') + tr;

  // Превью: показываем расчётное значение ESC при круиз-скорости
  const speed = parseInt(document.getElementById('cruiseSpeed')?.value || 1650);
  const pL = document.getElementById('previewL');
  const pR = document.getElementById('previewR');
  if (pL) pL.textContent = Math.min(2000, Math.max(1000, speed + tl));
  if (pR) pR.textContent = Math.min(2000, Math.max(1000, speed + tr));
}
function saveSettings() {
  const get=id=>{const e=document.getElementById(id);return e?e.value:null;};
  const body={
    pidKp:         parseFloat(get('pidKp')),
    maxDiff:       parseInt(get('maxDiff')),
    bearingAlpha:  parseFloat(get('bearingAlpha')),
    navInterval:   parseInt(get('navInterval')),
    pidKi:         parseFloat(get('pidKi')),
    pidKd:         parseFloat(get('pidKd')),
    pidCurve:      parseFloat(get('pidCurve')),
    cruiseGain:    parseFloat(get('cruiseGain')),
    cruiseSpeed:   parseInt(get('cruiseSpeed')),
    slowdownDist:  parseFloat(get('slowdownDist') || 5),
    slowdownSpeed: parseInt(get('slowdownSpeed')  || 1550),
    arrivalRadius: parseFloat(get('arrivalRadius')),
    minSatellites: parseInt(get('minSatellites')),
    trimLeft:      parseInt(get('trimLeft')  || 0),
    trimRight:     parseInt(get('trimRight') || 0),
  };
  fetch('/api/settings',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)})
    .then(r=>r.json()).then(d=>alert(d.ok?'✅ Сохранено!':'❌ Ошибка'))
    .catch(()=>alert('❌ Нет связи'));
}
function saveCompass() {
  const get=id=>{const e=document.getElementById(id);return e?e.value:null;};
  const body={
    compassDecl:     parseFloat(get('compassDecl')||0),
    compassAxis:     parseInt(get('compassAxis')||1),
    compassDeadzone: parseFloat(get('compassDeadzone')||2),
  };
  fetch('/api/settings',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)})
    .catch(()=>{});
}
function startCalib() {
  const btn=document.getElementById('calibBtn');
  if(btn) btn.disabled=true;
  fetch('/api/calibrate',{method:'POST'})
    .catch(()=>{ if(btn) btn.disabled=false; });
}

// ── Joystick page ────────────────────────────────────────────────
function tmplJoystick() {
  return `
<div class="card">
  <h2>Ручное управление</h2>
  <div id="joyRcWarn" style="display:none;background:#7f1d1d;color:#fca5a5;border-radius:6px;padding:8px 10px;font-size:.82rem;margin-bottom:10px;text-align:center">
    ⚠️ Пульт активен — джойстик заблокирован. Верни стики ch1/ch2 в центр.
  </div>
  <p style="font-size:.8rem;color:#64748b;margin:0 0 12px">Отпусти палец — корабль останавливается. Пульт имеет приоритет.</p>
  <div style="display:flex;justify-content:center;margin:4px 0 12px">
    <div id="joyWrap">
      <div id="joyStick"></div>
      <div class="joy-cross-v"></div>
      <div class="joy-cross-h"></div>
    </div>
  </div>
  <div style="display:flex;justify-content:center;gap:24px;font-size:.85rem;color:#64748b">
    <span>Лев: <b id="joyL" style="color:#e2e8f0">1500</b></span>
    <span>Прав: <b id="joyR" style="color:#e2e8f0">1500</b></span>
  </div>
</div>`;
}

// Джойстик — логика
let joyActive = false;
let joyTimerId = null;
let joyL = 1500, joyR = 1500;

function initJoystick() {
  const wrap  = document.getElementById('joyWrap');
  const stick = document.getElementById('joyStick');
  if (!wrap || !stick) return;

  const ZONE = 67;   // радиус рабочей зоны (px, круг 200px)
  const SR   = 33;   // радиус кружка стика

  function getOffset(e) {
    const rect = wrap.getBoundingClientRect();
    const cx = rect.left + rect.width  / 2;
    const cy = rect.top  + rect.height / 2;
    const src = e.touches ? e.touches[0] : e;
    let dx = src.clientX - cx;
    let dy = src.clientY - cy;   // положительный = вниз по экрану
    const d = Math.sqrt(dx*dx + dy*dy);
    if (d > ZONE) { dx = dx/d*ZONE; dy = dy/d*ZONE; }
    return {dx, dy};
  }

  function apply({dx, dy}) {
    // Позиция стика: центр круга при dx=0,dy=0 → left=74, top=74
    stick.style.left = (110 - SR + dx) + 'px';
    stick.style.top  = (110 - SR + dy) + 'px';   // dy>0 = вниз = назад

    // dy: палец вниз (dy>0) = назад (fwd<0), палец вверх (dy<0) = вперёд (fwd>0)
    const fwd  = Math.round(-dy / ZONE * 400);  // -400..+400
    const turn = Math.round( dx / ZONE * 300);  // -300..+300
    joyL = clamp(1500 + fwd + turn, 1000, 2000);
    joyR = clamp(1500 + fwd - turn, 1000, 2000);
    setText('joyL', joyL);
    setText('joyR', joyR);
  }

  function clamp(v, mn, mx) { return Math.max(mn, Math.min(mx, v)); }

  function sendNow() {
    fetch('/api/joystick', {
      method: 'POST',
      headers: {'Content-Type':'application/json'},
      body: `{"l":${joyL},"r":${joyR}}`
    }).then(r => r.json()).then(d => {
      const warn = document.getElementById('joyRcWarn');
      if (warn) warn.style.display = (d.ok === false && d.reason === 'rc_active') ? '' : 'none';
    }).catch(()=>{});
  }

  function onStart(e) {
    e.preventDefault();
    joyActive = true;
    stick.style.background = '#1d4ed8';
    apply(getOffset(e));
    sendNow();
    // Шлём каждые 100мс пока держим
    joyTimerId = setInterval(sendNow, 100);
  }

  function onMove(e) {
    e.preventDefault();
    if (!joyActive) return;
    apply(getOffset(e));
    // НЕ шлём здесь — interval сам отправит свежие joyL/joyR
  }

  function onEnd(e) {
    if (!joyActive) return;
    joyActive = false;
    clearInterval(joyTimerId);
    joyTimerId = null;
    stick.style.left = '67px';
    stick.style.top  = '67px';
    stick.style.background = '#3b82f6';
    joyL = 1500; joyR = 1500;
    setText('joyL', 1500); setText('joyR', 1500);
    // Стоп
    fetch('/api/joystick', {
      method: 'POST',
      headers: {'Content-Type':'application/json'},
      body: '{"l":1500,"r":1500}'
    });
    const warn = document.getElementById('joyRcWarn');
    if (warn) warn.style.display = 'none';
  }

  wrap.addEventListener('touchstart',  onStart, {passive:false});
  wrap.addEventListener('touchmove',   onMove,  {passive:false});
  wrap.addEventListener('touchend',    onEnd,   {passive:false});
  wrap.addEventListener('touchcancel', onEnd,   {passive:false});
  wrap.addEventListener('mousedown',   onStart);
  document.addEventListener('mousemove', e => { if(joyActive) onMove(e); });
  document.addEventListener('mouseup',   e => { if(joyActive) onEnd(e); });
}

// ── Log page ─────────────────────────────────────────────────────
function tmplLog() {
  return `
<div class="card" style="padding-bottom:6px">
  <div style="display:flex;align-items:center;justify-content:space-between;margin-bottom:8px">
    <h2 style="margin:0">Лог событий</h2>
    <button class="btn btn-blue" style="padding:4px 12px;font-size:.8rem" onclick="loadLog()">↻ Обновить</button>
  </div>
  <div id="logList" style="font-size:.78rem;font-family:monospace;line-height:1.7;color:var(--text-secondary)">Загрузка...</div>
</div>
<div class="card">
  <h2>Лог навигации AUTO (CSV)</h2>
  <p style="font-size:.8rem;color:#64748b;margin:0 0 10px">
    Пишется автоматически, пока корабль реально едет в AUTO (включая авто-возврат домой при потере пульта):
    курс, ошибка, стадии PID, PWM моторов, GPS-трек. Кольцевой буфер — старые записи затираются новыми.
  </p>
  <div class="row"><span>Записей в буфере</span><span id="nl-count">—</span></div>
  <div style="display:flex;gap:8px;margin-top:10px">
    <a class="btn btn-blue" href="/api/navlog.csv" download="navlog.csv" style="flex:1;text-align:center;text-decoration:none">⬇ Скачать CSV</a>
    <button class="btn" style="flex:1;background:#ef4444" onclick="clearNavLog()">🗑 Очистить</button>
  </div>
</div>`;
}

function loadNavLogStatus() {
  fetch('/api/navlog/status')
    .then(r=>r.json())
    .then(d=>setText('nl-count', (d.count ?? 0) + ' / ' + (d.capacity ?? 0)))
    .catch(()=>setText('nl-count','—'));
}
function clearNavLog() {
  if (!confirm('Очистить лог навигации AUTO?')) return;
  fetch('/api/navlog/clear', {method:'POST'})
    .then(()=>loadNavLogStatus())
    .catch(()=>alert('Ошибка'));
}

function loadLog() {
  fetch('/api/log')
    .then(r=>r.json())
    .then(entries=>{
      const el = document.getElementById('logList');
      if (!el) return;
      if (!entries.length) { el.innerHTML='<i>Лог пуст</i>'; return; }
      el.innerHTML = entries.map(e=>{
        const sec = (e.ms/1000).toFixed(1);
        const t   = `<span style="color:#64748b;min-width:60px;display:inline-block">[${sec}s]</span>`;
        // цвет по ключевым словам
        let color = 'var(--text-secondary)';
        if (/arrived|OK|saved|started|connected/i.test(e.msg)) color='#10b981';
        if (/LOST|lost|too far|STOP|stopped/i.test(e.msg))     color='#f59e0b';
        if (/error|fail/i.test(e.msg))                         color='#ef4444';
        return `<div>${t} <span style="color:${color}">${e.msg}</span></div>`;
      }).join('');
    })
    .catch(()=>{
      const el=document.getElementById('logList');
      if(el) el.textContent='Ошибка загрузки';
    });
}


buildPage();
connectWS();

function tmplOta() {
  return `
<div class="card">
  <h2>Обновление прошивки</h2>
  <p style="font-size:.85rem;color:#64748b;margin-bottom:12px">
    Скомпилируй прошивку в PlatformIO (Ctrl+Alt+B), возьми файл
    <b>.pio/build/esp32doit-devkit-v1/firmware.bin</b> и загрузи здесь.
  </p>
  <input type="file" id="otaFile" accept=".bin" style="margin-bottom:12px"/>
  <br/>
  <button id="otaBtn" onclick="otaUpload()">Загрузить прошивку</button>
  <div id="otaStatus" style="margin-top:12px;font-size:.9rem"></div>
  <div id="otaBar" style="display:none;margin-top:8px;background:#1e293b;border-radius:6px;height:12px">
    <div id="otaFill" style="height:100%;border-radius:6px;background:#22c55e;width:0%;transition:width .3s"></div>
  </div>
</div>`;
}

function otaUpload() {
  const file = document.getElementById('otaFile').files[0];
  if (!file) { alert('Выбери .bin файл'); return; }
  const btn = document.getElementById('otaBtn');
  const status = document.getElementById('otaStatus');
  const bar = document.getElementById('otaBar');
  const fill = document.getElementById('otaFill');
  btn.disabled = true;
  bar.style.display = 'block';
  status.textContent = 'Загрузка...';
  const xhr = new XMLHttpRequest();
  xhr.open('POST', '/update');
  xhr.upload.onprogress = e => {
    if (e.lengthComputable) {
      const pct = Math.round(e.loaded * 100 / e.total);
      fill.style.width = pct + '%';
      status.textContent = 'Загрузка... ' + pct + '%';
    }
  };
  xhr.onload = () => {
    if (xhr.responseText === 'OK') {
      status.textContent = '✅ Готово! ESP32 перезагружается...';
      fill.style.width = '100%';
    } else {
      status.textContent = '❌ Ошибка: ' + xhr.responseText;
      btn.disabled = false;
    }
  };
  xhr.onerror = () => {
    status.textContent = '❌ Нет соединения';
    btn.disabled = false;
  };
  const fd = new FormData();
  fd.append('firmware', file, file.name);
  xhr.send(fd);
}


function gpsReset() {
  if (!confirm('Сбросить GPS модуль? Координаты Лимы уйдут, но поиск спутников займёт 1-2 минуты.')) return;
  fetch('/api/gpsreset', {method:'POST'})
    .then(r => r.text())
    .then(() => alert('GPS сброшен — подожди 1-2 минуты для поиска спутников'))
    .catch(() => alert('Ошибка'));
}
