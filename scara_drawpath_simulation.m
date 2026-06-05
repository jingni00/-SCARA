% SCARA draw-path interpolation simulation.
% This script mirrors the DRAW1 path and trapezoidal planner in Core/Src/main.c.

activeLink = 110.0;       % mm
passiveLink = 220.0;      % mm
motorDistance = 160.0;    % mm, motors at (-80, 0) and (80, 0)
stepsPerRev = 3062;       % pulse/rev
feed = 40.0;              % mm/s
accel = 120.0;            % mm/s^2
cornerRadius = 0.8;       % mm
dt = 0.001;               % s, same as DDA_TICK_US = 1000 us

nominalBlocks = buildDrawPath(cornerRadius);
pathOffset = findBestPathOffset(nominalBlocks, activeLink, passiveLink, motorDistance);
blocks = shiftBlocks(nominalBlocks, pathOffset);
pathLength = sum([blocks.length]);
[t, s, v, a] = trapezoidProfile(pathLength, feed, accel, dt);
xy = samplePath(blocks, s);
[nominalMinMargin, nominalMeanMargin] = workspaceMargins(samplePath(nominalBlocks, linspace(0, pathLength, 500).'), ...
    activeLink, passiveLink, motorDistance);
[bestMinMargin, bestMeanMargin] = workspaceMargins(samplePath(blocks, linspace(0, pathLength, 500).'), ...
    activeLink, passiveLink, motorDistance);
[theta1, theta2, motor1Steps, motor2Steps, ikOk] = scaraIK( ...
    xy(:, 1), xy(:, 2), activeLink, passiveLink, motorDistance, stepsPerRev);

fprintf('轨迹长度：%.3f mm\n', pathLength);
fprintf('推荐整体平移：X %.2f mm，Y %.2f mm\n', pathOffset(1), pathOffset(2));
fprintf('推荐绘图起点：X %.2f mm，Y %.2f mm\n', pathOffset(1), 250.1 + pathOffset(2));
fprintf('原位置最小/平均工作空间余量：%.2f / %.2f mm\n', nominalMinMargin, nominalMeanMargin);
fprintf('推荐位置最小/平均工作空间余量：%.2f / %.2f mm\n', bestMinMargin, bestMeanMargin);
fprintf('总时间：%.3f s\n', t(end));
fprintf('峰值速度：%.3f mm/s\n', max(v));
fprintf('逆解有效采样点：%d / %d\n', nnz(ikOk), numel(ikOk));

figure('Name', '工作空间与插补绘图轨迹');
tiledlayout(1, 1);
nexttile;
plotWorkspace(activeLink, passiveLink, motorDistance);
hold on;
hOriginal = plotBlockPath(nominalBlocks, 0.5, [0.60 0.60 0.60], 1.0, '--');
hPath = plotBlockPath(blocks, 0.5, [0.95 0.35 0.15], 2.2, '-');
sampleEvery = max(1, floor(numel(t) / 180));
hSamples = plot(xy(1:sampleEvery:end, 1), xy(1:sampleEvery:end, 2), '.', ...
    'Color', [0.05 0.35 0.95], 'MarkerSize', 7);
rawPts = [0 250.1; 0 230.1; -15 230.1; -15 210.1; -45 180.1; ...
    -20 180.1; 20 180.1; 45 180.1; 15 210.1; 15 230.1; 0 230.1];
rawPts = rawPts + pathOffset;
hRaw = plot(rawPts(:, 1), rawPts(:, 2), 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 4);
hMotors = plot([-motorDistance / 2, motorDistance / 2], [0, 0], 'ks', ...
    'MarkerFaceColor', 'k', 'MarkerSize', 6);
title('并联 SCARA 工作空间与 DRAW1 插补轨迹');
xlabel('X (mm)');
ylabel('Y (mm)');
legend([hOriginal, hPath, hSamples, hRaw, hMotors], '原始位置轨迹', '推荐位置轨迹', ...
    '插补采样点', '关键路径点', '电机位置', 'Location', 'southoutside');
axis equal;
xlim([-230 230]);
ylim([0 340]);
grid on;

figure('Name', '梯形速度曲线');
tiledlayout(3, 1);
nexttile;
plot(t, s, 'LineWidth', 1.4);
grid on;
ylabel('s (mm)');
title('弧长指令 s(t)');
nexttile;
plot(t, v, 'LineWidth', 1.4);
grid on;
ylabel('v (mm/s)');
title('进给速度曲线');
nexttile;
plot(t, a, 'LineWidth', 1.4);
grid on;
xlabel('Time (s)');
ylabel('a (mm/s^2)');
title('加速度曲线');

figure('Name', '笛卡尔与关节插补结果');
tiledlayout(2, 2);
nexttile;
plot(t, xy(:, 1), 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('X (mm)');
title('X 方向插补');
nexttile;
plot(t, xy(:, 2), 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Y (mm)');
title('Y 方向插补');
nexttile;
plot(t, rad2deg(theta1), t, rad2deg(theta2), 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Angle (deg)');
title('逆运动学得到的电机摆臂角度');
legend('电机 1', '电机 2', 'Location', 'best');
nexttile;
plot(t, motor1Steps, t, motor2Steps, 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Step count');
title('等效步进电机脉冲数');
legend('电机 1 步数', '电机 2 步数', 'Location', 'best');

function blocks = buildDrawPath(cornerRadius)
    pre = [0 250.1; 0 230.1; -15 230.1; -15 210.1; -45 180.1; -20 180.1];
    tail = [20 180.1; 45 180.1; 15 210.1; 15 230.1; 0 230.1];
    blocks = emptyBlock();
    cursor = pre(1, :);
    for k = 2:size(pre, 1) - 1
        [newBlocks, cursor] = roundedCornerBlocks(pre(k - 1, :), pre(k, :), ...
            pre(k + 1, :), cursor, cornerRadius);
        blocks = appendBlocks(blocks, newBlocks);
    end
    blocks = appendBlocks(blocks, makeLine(cursor, pre(end, :)));
    cursor = pre(end, :);
    blocks = appendBlocks(blocks, makeArc(cursor, tail(1, :), [0 180], false));
    cursor = tail(1, :);
    for k = 2:size(tail, 1) - 1
        [newBlocks, cursor] = roundedCornerBlocks(tail(k - 1, :), tail(k, :), ...
            tail(k + 1, :), cursor, cornerRadius);
        blocks = appendBlocks(blocks, newBlocks);
    end
    blocks = appendBlocks(blocks, makeLine(cursor, tail(end, :)));
end

function bestOffset = findBestPathOffset(blocks, activeLink, passiveLink, motorDistance)
    pts = densePathPoints(blocks, 0.5);
    xCandidates = -80:1:80;
    yCandidates = -80:1:80;
    bestScore = -inf;
    bestOffset = [0 0];
    for ix = 1:numel(xCandidates)
        for iy = 1:numel(yCandidates)
            offset = [xCandidates(ix), yCandidates(iy)];
            [minMargin, meanMargin] = workspaceMargins(pts + offset, ...
                activeLink, passiveLink, motorDistance);
            score = minMargin + 0.02 * meanMargin - 0.001 * abs(offset(1));
            if score > bestScore
                bestScore = score;
                bestOffset = offset;
            end
        end
    end
    fineX = bestOffset(1) - 2:0.1:bestOffset(1) + 2;
    fineY = bestOffset(2) - 2:0.1:bestOffset(2) + 2;
    for ix = 1:numel(fineX)
        for iy = 1:numel(fineY)
            offset = [fineX(ix), fineY(iy)];
            [minMargin, meanMargin] = workspaceMargins(pts + offset, ...
                activeLink, passiveLink, motorDistance);
            score = minMargin + 0.02 * meanMargin - 0.001 * abs(offset(1));
            if score > bestScore
                bestScore = score;
                bestOffset = offset;
            end
        end
    end
end

function [minMargin, meanMargin] = workspaceMargins(points, activeLink, passiveLink, motorDistance)
    halfD = motorDistance / 2;
    rMin = abs(passiveLink - activeLink);
    rMax = activeLink + passiveLink;
    rLeft = hypot(points(:, 1) + halfD, points(:, 2));
    rRight = hypot(points(:, 1) - halfD, points(:, 2));
    margin = min([rLeft - rMin, rMax - rLeft, rRight - rMin, rMax - rRight], [], 2);
    minMargin = min(margin);
    meanMargin = mean(margin);
end

function shifted = shiftBlocks(blocks, offset)
    shifted = blocks;
    for k = 1:numel(shifted)
        shifted(k).p0 = shifted(k).p0 + offset;
        shifted(k).p1 = shifted(k).p1 + offset;
        if all(isfinite(shifted(k).center))
            shifted(k).center = shifted(k).center + offset;
            shifted(k).a0 = atan2(shifted(k).p0(2) - shifted(k).center(2), ...
                shifted(k).p0(1) - shifted(k).center(1));
        end
    end
end

function [newBlocks, cursor] = roundedCornerBlocks(prev, corner, next, cursor, radius)
    minSegment = 0.10;
    lenIn = norm(corner - prev);
    lenOut = norm(next - corner);
    if lenIn < minSegment || lenOut < minSegment
        newBlocks = makeLine(cursor, corner);
        cursor = corner;
        return;
    end
    inDir = (corner - prev) / lenIn;
    outDir = (next - corner) / lenOut;
    cosTheta = clamp(dot(-inDir, outDir), -0.999, 0.999);
    theta = acos(cosTheta);
    trim = radius / tan(theta * 0.5);
    if theta < 0.05 || trim <= 0
        newBlocks = makeLine(cursor, corner);
        cursor = corner;
        return;
    end
    trim = clamp(trim, 0, min(lenIn, lenOut) * 0.45);
    if trim < minSegment
        newBlocks = makeLine(cursor, corner);
        cursor = corner;
        return;
    end
    t1 = corner - inDir * trim;
    t2 = corner + outDir * trim;
    n1 = [-inDir(2), inDir(1)];
    n2 = [-outDir(2), outDir(1)];
    denom = cross2(n1, n2);
    newBlocks = emptyBlock();
    newBlocks = appendBlocks(newBlocks, makeLine(cursor, t1));
    if abs(denom) < 1e-4
        newBlocks = appendBlocks(newBlocks, makeLine(t1, t2));
        cursor = t2;
        return;
    end
    center = t1 + n1 * (cross2(t2 - t1, n2) / denom);
    cw = cross2(inDir, outDir) < 0;
    newBlocks = appendBlocks(newBlocks, makeArc(t1, t2, center, cw));
    cursor = t2;
end

function block = makeLine(p0, p1)
    len = norm(p1 - p0);
    block = struct('type', 'line', 'p0', p0, 'p1', p1, 'center', [NaN NaN], ...
        'radius', NaN, 'a0', NaN, 'sweep', NaN, 'length', len);
    if len < 0.10
        block = emptyBlock();
    end
end

function block = makeArc(p0, p1, center, cw)
    r0 = norm(p0 - center);
    r1 = norm(p1 - center);
    if r0 < 0.10 || abs(r0 - r1) > 1.0
        block = emptyBlock();
        return;
    end
    a0 = atan2(p0(2) - center(2), p0(1) - center(1));
    a1 = atan2(p1(2) - center(2), p1(1) - center(1));
    sweep = a1 - a0;
    if cw
        if sweep >= 0
            sweep = sweep - 2 * pi;
        end
    else
        if sweep <= 0
            sweep = sweep + 2 * pi;
        end
    end
    radius = 0.5 * (r0 + r1);
    block = struct('type', 'arc', 'p0', p0, 'p1', p1, 'center', center, ...
        'radius', radius, 'a0', a0, 'sweep', sweep, 'length', abs(radius * sweep));
end

function blocks = emptyBlock()
    blocks = struct('type', {}, 'p0', {}, 'p1', {}, 'center', {}, ...
        'radius', {}, 'a0', {}, 'sweep', {}, 'length', {});
end

function out = appendBlocks(out, add)
    if isempty(add)
        return;
    end
    out = [out, add]; %#ok<AGROW>
end

function [t, s, v, a] = trapezoidProfile(pathLength, feed, accel, dt)
    vmax = clamp(feed, 0.1, 1000.0);
    acc = clamp(accel, 1.0, 10000.0);
    tAccel = vmax / acc;
    sAccel = 0.5 * acc * tAccel ^ 2;
    if 2 * sAccel >= pathLength
        tAccel = sqrt(pathLength / acc);
        tFlat = 0;
        vmax = acc * tAccel;
    else
        tFlat = (pathLength - 2 * sAccel) / vmax;
    end
    totalTime = 2 * tAccel + tFlat;
    t = (0:dt:totalTime).';
    if t(end) < totalTime
        t(end + 1, 1) = totalTime;
    end
    s = zeros(size(t));
    v = zeros(size(t));
    a = zeros(size(t));
    sAccel = 0.5 * acc * tAccel ^ 2;
    sFlat = vmax * tFlat;
    for k = 1:numel(t)
        tk = t(k);
        if tk < tAccel
            s(k) = 0.5 * acc * tk ^ 2;
            v(k) = acc * tk;
            a(k) = acc;
        elseif tk < tAccel + tFlat
            s(k) = sAccel + vmax * (tk - tAccel);
            v(k) = vmax;
            a(k) = 0;
        elseif tk < totalTime
            td = tk - tAccel - tFlat;
            s(k) = sAccel + sFlat + vmax * td - 0.5 * acc * td ^ 2;
            v(k) = vmax - acc * td;
            a(k) = -acc;
        else
            s(k) = pathLength;
            v(k) = 0;
            a(k) = 0;
        end
    end
end

function xy = samplePath(blocks, s)
    cumLen = [0, cumsum([blocks.length])];
    xy = zeros(numel(s), 2);
    for k = 1:numel(s)
        sk = min(max(s(k), 0), cumLen(end));
        idx = find(cumLen(2:end) >= sk, 1, 'first');
        if isempty(idx)
            idx = numel(blocks);
        end
        localS = sk - cumLen(idx);
        xy(k, :) = blockPointAt(blocks(idx), localS);
    end
end

function p = blockPointAt(block, localS)
    if block.length <= 1e-9
        p = block.p1;
        return;
    end
    u = clamp(localS / block.length, 0, 1);
    if strcmp(block.type, 'arc')
        ang = block.a0 + block.sweep * u;
        p = block.center + block.radius * [cos(ang), sin(ang)];
    else
        p = block.p0 + (block.p1 - block.p0) * u;
    end
end

function h = plotBlockPath(blocks, ds, color, width, style)
    pts = densePathPoints(blocks, ds);
    h = plot(pts(:, 1), pts(:, 2), style, 'Color', color, 'LineWidth', width);
end

function pts = densePathPoints(blocks, ds)
    pts = [];
    for k = 1:numel(blocks)
        n = max(2, ceil(blocks(k).length / ds));
        localS = linspace(0, blocks(k).length, n).';
        blockPts = zeros(n, 2);
        for j = 1:n
            blockPts(j, :) = blockPointAt(blocks(k), localS(j));
        end
        pts = [pts; blockPts]; %#ok<AGROW>
    end
end

function plotWorkspace(activeLink, passiveLink, motorDistance)
    x = linspace(-240, 240, 361);
    y = linspace(0, 340, 281);
    [X, Y] = meshgrid(x, y);
    halfD = motorDistance / 2;
    rLeft = hypot(X + halfD, Y);
    rRight = hypot(X - halfD, Y);
    rMin = abs(passiveLink - activeLink);
    rMax = passiveLink + activeLink;
    reachable = rLeft >= rMin & rLeft <= rMax & rRight >= rMin & rRight <= rMax;
    imagesc(x, y, reachable);
    set(gca, 'YDir', 'normal');
    colormap(gca, [1 1 1; 0.88 0.94 1.00]);
    hold on;
    contour(X, Y, double(reachable), [0.5 0.5], 'Color', [0.2 0.45 0.8], 'LineWidth', 1.0);
end

function [theta1, theta2, m1, m2, ok] = scaraIK(x, y, activeLink, passiveLink, motorDistance, stepsPerRev)
    halfD = motorDistance / 2;
    dxLeft = x + halfD;
    dxRight = x - halfD;
    rLeft = hypot(dxLeft, y);
    rRight = hypot(dxRight, y);
    rMin = abs(passiveLink - activeLink);
    rMax = passiveLink + activeLink;
    ok = rLeft > 1e-4 & rRight > 1e-4 & ...
        rLeft >= rMin & rLeft <= rMax & rRight >= rMin & rRight <= rMax;
    cLeft = ((activeLink ^ 2) + (rLeft .^ 2) - (passiveLink ^ 2)) ./ ...
        (2 * activeLink .* rLeft);
    cRight = ((activeLink ^ 2) + (rRight .^ 2) - (passiveLink ^ 2)) ./ ...
        (2 * activeLink .* rRight);
    cLeft = clamp(cLeft, -1, 1);
    cRight = clamp(cRight, -1, 1);
    theta1 = atan2(y, dxLeft) + acos(cLeft);
    theta2 = atan2(y, dxRight) - acos(cRight);
    theta1(~ok) = NaN;
    theta2(~ok) = NaN;
    m1 = round((theta1 / (2 * pi)) * stepsPerRev);
    m2 = round((theta2 / (2 * pi)) * stepsPerRev);
end

function y = clamp(x, lo, hi)
    y = min(max(x, lo), hi);
end

function c = cross2(a, b)
    c = a(1) * b(2) - a(2) * b(1);
end
