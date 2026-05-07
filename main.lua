--[[
  4D Gyro Toggle — Widget Ethos (v4)
-- -----------------------------------------------------------------
-- Constante
-- -----------------------------------------------------------------
local SENSOR_APPID   = 0x0C30  -- ID sensor FrSky stabilizer config
local REG_CHINV_G1   = 0xA9    -- sub1=AIL, sub2=ELE, sub3=RUD
local REG_CHINV_G2   = 0xAA    -- sub1=AIL2, sub2=ELE2
local REG_STAB_GAIN  = 0xAB    -- sub1=AIL, sub2=ELE, sub3=RUD (0-200)
local VAL_INV_ON     = 0xFF
local VAL_INV_OFF    = 0x00
local GAIN_MIN       = 0
local GAIN_MAX       = 200

-- Modules RF
local MOD_INTERNAL   = 0x00
local MOD_EXTERNAL   = 0x01

-- -----------------------------------------------------------------
-- protocol
-- -----------------------------------------------------------------
local ST_INIT        = 0  -- doit demarrer la lecture baseline
local ST_READ_G1     = 1  -- attend reponse de 0xA9
local ST_READ_G2     = 2  -- attend reponse de 0xAA
local ST_READ_G3     = 3  -- attend reponse de 0xAB (gain)
local ST_READY       = 4  -- baseline connu, monitore le switch
local ST_WRITING     = 5  -- ecrit les registres, en attente

local READ_TIMEOUT   = 2.0  -- secondes avant retry lecture
local WRITE_TIMEOUT  = 1.0  -- secondes avant retry ecriture

-- -----------------------------------------------------------------
-- 
-- -----------------------------------------------------------------
local sensor = nil

local function getSensor()
  if sensor == nil then
    sensor = sport.getSensor(SENSOR_APPID)
  end
  return sensor
end

-- Packe 3 octets en un mot 24 bits (format du protocole)
local function pack3(d1, d2, d3)
  return (d1 & 0xFF) | ((d2 & 0xFF) << 8) | ((d3 & 0xFF) << 16)
end

-- Depacke les 3 octets
local function unpack3(v)
  v = v or 0
  return v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF
end

-- Normalise: tout ce qui n'est pas 0 devient 0xFF
local function norm(v)
  return (v ~= 0) and VAL_INV_ON or VAL_INV_OFF
end

-- Flippe une valeur d'invert (0 <-> 0xFF) seulement si enable
local function flipIf(enable, v)
  if not enable then return norm(v) end
  return (norm(v) == VAL_INV_OFF) and VAL_INV_ON or VAL_INV_OFF
end

-- Clamp un gain dans la plage 0-200
local function clampGain(v)
  if v < GAIN_MIN then return GAIN_MIN end
  if v > GAIN_MAX then return GAIN_MAX end
  return v
end

-- -----------------------------------------------------------------
-- Widget lifecycle
-- -----------------------------------------------------------------
local function name()
  return "4D Gyro"
end

local function create()
  return {
    -- configuration (persistee)
    switchSrc      = nil,    -- source switch assignee
    reverseOnHigh  = true,   -- true: switch actif = reculons, false: inverse
    ailCount       = 1,      -- 0=aucun, 1=AIL, 2=AIL+AIL2
    eleCount       = 1,      -- 0=aucun, 1=ELE, 2=ELE+ELE2
    rudCount       = 1,      -- 0=aucun, 1=RUD
    rfModule       = MOD_INTERNAL,
    ailRevGain     = 100,    -- gain gyro AIL en mode REV (0-200)
    eleRevGain     = 100,    -- gain gyro ELE en mode REV (0-200)
    rudRevGain     = 100,    -- gain gyro RUD en mode REV (0-200)

    -- etat runtime (non persiste)
    state          = ST_INIT,
    stateTimer     = 0,
    baselineG1     = nil,    -- valeur 24-bit lue de 0xA9 (CHInvert FWD)
    baselineG2     = nil,    -- valeur 24-bit lue de 0xAA (CHInvert2 FWD)
    baselineGain   = nil,    -- valeur 24-bit lue de 0xAB (gains FWD)
    currentMode    = nil,    -- "FWD" ou "REV"
    lastError      = nil,
    writeQueue     = {},     -- liste de {addr, value}
    lastSensorOk   = 0,      -- timestamp derniere com OK
  }
end

-- -----------------------------------------------------------------
-- Calcul mode
-- -----------------------------------------------------------------
local function computeWrites(w, targetMode)
  local writes = {}
  local isRev = (targetMode == "REV")

  -- Groupe 1 INVERT: AIL / ELE / RUD
  if (w.ailCount >= 1) or (w.eleCount >= 1) or (w.rudCount >= 1) then
    local ail, ele, rud = unpack3(w.baselineG1 or 0)
    ail = isRev and flipIf(w.ailCount >= 1, ail) or norm(ail)
    ele = isRev and flipIf(w.eleCount >= 1, ele) or norm(ele)
    rud = isRev and flipIf(w.rudCount >= 1, rud) or norm(rud)
    writes[#writes + 1] = { REG_CHINV_G1, pack3(ail, ele, rud) }
  end

  -- Groupe 2 INVERT: AIL2 / ELE2
  if (w.ailCount >= 2) or (w.eleCount >= 2) then
    local ail2, ele2, _ = unpack3(w.baselineG2 or 0)
    ail2 = isRev and flipIf(w.ailCount >= 2, ail2) or norm(ail2)
    ele2 = isRev and flipIf(w.eleCount >= 2, ele2) or norm(ele2)
    writes[#writes + 1] = { REG_CHINV_G2, pack3(ail2, ele2, 0) }
  end

  -- GAINS de stabilisation: AIL / ELE / RUD (registre unique 0xAB)
  -- En FWD: on remet la baseline. En REV: on applique les gains
  -- configures pour les axes actives, et on garde la baseline pour
  -- les axes desactives (count=0).
  if (w.ailCount >= 1) or (w.eleCount >= 1) or (w.rudCount >= 1) then
    local ailG, eleG, rudG = unpack3(w.baselineGain or 0)
    if isRev then
      if w.ailCount >= 1 then ailG = clampGain(w.ailRevGain) end
      if w.eleCount >= 1 then eleG = clampGain(w.eleRevGain) end
      if w.rudCount >= 1 then rudG = clampGain(w.rudRevGain) end
    end
    writes[#writes + 1] = { REG_STAB_GAIN, pack3(ailG, eleG, rudG) }
  end

  return writes
end

local function resetBaseline(w)
  w.state        = ST_INIT
  w.baselineG1   = nil
  w.baselineG2   = nil
  w.baselineGain = nil
  w.currentMode  = nil
  w.writeQueue   = {}
  w.lastError    = nil
end

-- -----------------------------------------------------------------
-- config widget
-- -----------------------------------------------------------------
local function configure(w)
  local line

  line = form.addLine("Switch de mode")
  form.addSourceField(line, nil,
    function() return w.switchSrc end,
    function(v) w.switchSrc = v; resetBaseline(w) end)

  line = form.addLine("Switch actif =")
  form.addChoiceField(line, nil,
    { { "Reculons", 1 }, { "Marche avant", 0 } },
    function() return w.reverseOnHigh and 1 or 0 end,
    function(v) w.reverseOnHigh = (v == 1) end)

  line = form.addLine("Ailerons")
  form.addChoiceField(line, nil,
    { { "Non utilise", 0 }, { "1 canal (AIL)", 1 }, { "2 canaux (AIL+AIL2)", 2 } },
    function() return w.ailCount end,
    function(v) w.ailCount = v; resetBaseline(w) end)

  line = form.addLine("Profondeur")
  form.addChoiceField(line, nil,
    { { "Non utilise", 0 }, { "1 canal (ELE)", 1 }, { "2 canaux (ELE+ELE2)", 2 } },
    function() return w.eleCount end,
    function(v) w.eleCount = v; resetBaseline(w) end)

  line = form.addLine("Gouverne de direction")
  form.addChoiceField(line, nil,
    { { "Non utilise", 0 }, { "1 canal (RUD)", 1 } },
    function() return w.rudCount end,
    function(v) w.rudCount = v; resetBaseline(w) end)

  line = form.addLine("Module RF")
  form.addChoiceField(line, nil,
    { { "Interne", MOD_INTERNAL }, { "Externe", MOD_EXTERNAL } },
    function() return w.rfModule end,
    function(v)
      w.rfModule = v
      local s = getSensor()
      if s then s:module(v) end
      resetBaseline(w)
    end)

  -- ---- Gains gyro en mode RECULONS ----
  line = form.addLine("--- Gains gyro REV ---")

  line = form.addLine("Gain AIL reculons")
  local f
  f = form.addNumberField(line, nil, GAIN_MIN, GAIN_MAX,
    function() return w.ailRevGain end,
    function(v) w.ailRevGain = v end)
  f:suffix("%")
  f:enableInstantChange(false)

  line = form.addLine("Gain ELE reculons")
  f = form.addNumberField(line, nil, GAIN_MIN, GAIN_MAX,
    function() return w.eleRevGain end,
    function(v) w.eleRevGain = v end)
  f:suffix("%")
  f:enableInstantChange(false)

  line = form.addLine("Gain RUD reculons")
  f = form.addNumberField(line, nil, GAIN_MIN, GAIN_MAX,
    function() return w.rudRevGain end,
    function(v) w.rudRevGain = v end)
  f:suffix("%")
  f:enableInstantChange(false)

  line = form.addLine("")
  form.addTextButton(line, nil, "Relire le RX", function()
    resetBaseline(w)
  end)
end

-- -----------------------------------------------------------------
-- Helper: (registre 0xAB)
-- -----------------------------------------------------------------
local function startReadGain(w, s, now)
  if s:requestParameter(REG_STAB_GAIN) then
    w.state      = ST_READ_G3
    w.stateTimer = now + READ_TIMEOUT
  else
    w.state      = ST_INIT  -- retry au prochain cycle
  end
end

-- -----------------------------------------------------------------
--  principale: protocol S.Port
-- -----------------------------------------------------------------
local function wakeup(w)
  local s = getSensor()
  if s == nil then return end

  local now = os.clock()

  -- -- Etat: demarrer la lecture du groupe 1
  if w.state == ST_INIT then
    if s:requestParameter(REG_CHINV_G1) then
      w.state      = ST_READ_G1
      w.stateTimer = now + READ_TIMEOUT
    end

  -- -- Etat: attendre reponse du groupe 1
  elseif w.state == ST_READ_G1 then
    local v = s:getParameter()
    if v ~= nil then
      if (v & 0xFF) == REG_CHINV_G1 then
        w.baselineG1   = (v >> 8) & 0xFFFFFF
        w.lastSensorOk = now
        -- Faut-il lire le groupe 2?
        if (w.ailCount >= 2) or (w.eleCount >= 2) then
          if s:requestParameter(REG_CHINV_G2) then
            w.state      = ST_READ_G2
            w.stateTimer = now + READ_TIMEOUT
          end
        else
          w.baselineG2 = 0
          startReadGain(w, s, now)
        end
      end
    elseif now > w.stateTimer then
      w.state     = ST_INIT
      w.lastError = "Timeout lecture G1"
    end

  -- -- Etat: attendre reponse du groupe 2
  elseif w.state == ST_READ_G2 then
    local v = s:getParameter()
    if v ~= nil then
      if (v & 0xFF) == REG_CHINV_G2 then
        w.baselineG2   = (v >> 8) & 0xFFFFFF
        w.lastSensorOk = now
        startReadGain(w, s, now)
      end
    elseif now > w.stateTimer then
      w.state     = ST_INIT
      w.lastError = "Timeout lecture G2"
    end

  -- -- Etat: attendre reponse du gain (groupe 3)
  elseif w.state == ST_READ_G3 then
    local v = s:getParameter()
    if v ~= nil then
      if (v & 0xFF) == REG_STAB_GAIN then
        w.baselineGain = (v >> 8) & 0xFFFFFF
        w.lastSensorOk = now
        w.state        = ST_READY
        w.lastError    = nil
      end
    elseif now > w.stateTimer then
      w.state     = ST_INIT
      w.lastError = "Timeout lecture gain"
    end

  -- -- Etat: pret, on monitore le switch
  elseif w.state == ST_READY then
    if w.switchSrc == nil then return end

    local raw = w.switchSrc:value()
    local active = (raw ~= nil) and (raw > 0)
    local desired
    if w.reverseOnHigh then
      desired = active and "REV" or "FWD"
    else
      desired = active and "FWD" or "REV"
    end

    if desired ~= w.currentMode then
      w.writeQueue = computeWrites(w, desired)
      w.targetMode = desired
      if #w.writeQueue > 0 then
        w.state      = ST_WRITING
        w.stateTimer = now + WRITE_TIMEOUT
      else
        w.currentMode = desired  -- rien a ecrire (aucune surface active)
      end
    end

  -- -- Etat: ecriture en cours
  elseif w.state == ST_WRITING then
    if #w.writeQueue == 0 then
      w.currentMode  = w.targetMode
      w.state        = ST_READY
      w.lastSensorOk = now
      lcd.invalidate()
      return
    end
    local pair = w.writeQueue[1]
    if s:writeParameter(pair[1], pair[2]) then
      table.remove(w.writeQueue, 1)
      w.stateTimer = now + WRITE_TIMEOUT
    elseif now > w.stateTimer then
      -- Echec: on force une relecture au prochain cycle
      w.lastError  = "Timeout ecriture"
      w.writeQueue = {}
      w.state      = ST_INIT
    end
  end
end

-- -----------------------------------------------------------------
-- disp
-- -----------------------------------------------------------------
local function paint(w)
  local ww, hh = lcd.getWindowSize()
  lcd.color(lcd.RGB(20, 20, 20))
  lcd.drawFilledRectangle(0, 0, ww, hh)

  -- Titre
  lcd.font(FONT_XS)
  lcd.color(lcd.RGB(160, 160, 160))
  lcd.drawText(6, 4, "4D GYRO")

  -- Etat principal
  lcd.font(FONT_XL)
  local msg, col
  if w.lastError then
    msg, col = w.lastError, lcd.RGB(255, 80, 80)
  elseif w.state == ST_INIT
      or w.state == ST_READ_G1
      or w.state == ST_READ_G2
      or w.state == ST_READ_G3 then
    msg, col = "Lecture RX...", lcd.RGB(255, 200, 0)
  elseif w.state == ST_WRITING then
    msg, col = "Ecriture...", lcd.RGB(255, 200, 0)
  elseif w.currentMode == "FWD" then
    msg, col = "AVANT", lcd.RGB(80, 220, 80)
  elseif w.currentMode == "REV" then
    msg, col = "RECULONS", lcd.RGB(255, 140, 0)
  else
    msg, col = "Pret", lcd.RGB(200, 200, 200)
  end

  lcd.color(col)
  local tw, th = lcd.getTextSize(msg)
  lcd.drawText((ww - tw) / 2, (hh - th) / 2, msg)

  -- Ligne info en bas
  if w.switchSrc == nil then
    lcd.font(FONT_XS)
    lcd.color(lcd.RGB(255, 140, 0))
    lcd.drawText(6, hh - 14, "Configure un switch")
  end
end

-- -----------------------------------------------------------------
-- stai config
-- -----------------------------------------------------------------
local function read(w)
  w.switchSrc     = storage.read("sw")
  w.reverseOnHigh = storage.read("rh")
  if w.reverseOnHigh == nil then w.reverseOnHigh = true end
  w.ailCount   = storage.read("ail") or 1
  w.eleCount   = storage.read("ele") or 1
  w.rudCount   = storage.read("rud") or 1
  w.rfModule   = storage.read("mod") or MOD_INTERNAL
  w.ailRevGain = storage.read("ag")  or 100
  w.eleRevGain = storage.read("eg")  or 100
  w.rudRevGain = storage.read("rg")  or 100
  -- Applique le module RF au sensor
  local s = getSensor()
  if s then s:module(w.rfModule) end
  return true
end

local function write(w)
  storage.write("sw",  w.switchSrc)
  storage.write("rh",  w.reverseOnHigh)
  storage.write("ail", w.ailCount)
  storage.write("ele", w.eleCount)
  storage.write("rud", w.rudCount)
  storage.write("mod", w.rfModule)
  storage.write("ag",  w.ailRevGain)
  storage.write("eg",  w.eleRevGain)
  storage.write("rg",  w.rudRevGain)
  return true
end

-- -----------------------------------------------------------------
-- reg du widget
-- -----------------------------------------------------------------
local function init()
  system.registerWidget({
    key       = "fd4dgy",   -- unique, < 8 chars
    name      = name,
    create    = create,
    configure = configure,
    wakeup    = wakeup,
    paint     = paint,
    read      = read,
    write     = write,
    title     = false,      -- on dessine notre propre titre
  })
end

return { init = init }
