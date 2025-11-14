// 🚀 FULL MONTY CHART ENHANCEMENTS

// Add annotation plugin configuration
const annotationPlugin = {
    id: 'customAnnotations',
    afterDatasetsDraw(chart) {
        const ctx = chart.ctx;
        const chartArea = chart.chartArea;
        const datasets = chart.data.datasets;
        
        if (!datasets[0].data || datasets[0].data.length === 0) return;
        
        const currentData = datasets[0].data;
        
        // Calculate metrics
        const peakCurrent = Math.max(...currentData.map(d => d.y));
        const riseTimeData = calculateRiseTime(currentData);
        const weldStartIdx = currentData.findIndex(d => d.x >= 0);
        
        // Draw rise time annotation
        if (riseTimeData) {
            drawRiseTimeMarker(ctx, chart, riseTimeData);
        }
        
        // Draw peak current line
        drawPeakLine(ctx, chart, peakCurrent);
        
        // Draw weld start marker (t=0)
        if (weldStartIdx >= 0) {
            drawWeldStartMarker(ctx, chart);
        }
        
        // Draw metrics overlay
        drawMetricsOverlay(ctx, chartArea, riseTimeData, peakCurrent);
    }
};

function calculateRiseTime(data) {
    // Find where current rises from ~0 to peak
    const threshold = 10; // 10A threshold for "start"
    const peakCurrent = Math.max(...data.map(d => d.y));
    const peakThreshold = peakCurrent * 0.9; // 90% of peak
    
    let startIdx = data.findIndex(d => d.y > threshold);
    let endIdx = data.findIndex(d => d.y > peakThreshold);
    
    if (startIdx >= 0 && endIdx > startIdx) {
        return {
            startTime: data[startIdx].x,
            endTime: data[endIdx].x,
            riseTime: data[endIdx].x - data[startIdx].x,
            startCurrent: data[startIdx].y,
            endCurrent: data[endIdx].y
        };
    }
    return null;
}

function drawRiseTimeMarker(ctx, chart, riseData) {
    const xScale = chart.scales.x;
    const yScale = chart.scales.y;
    
    const x1 = xScale.getPixelForValue(riseData.startTime);
    const x2 = xScale.getPixelForValue(riseData.endTime);
    const y1 = yScale.getPixelForValue(riseData.startCurrent);
    const y2 = yScale.getPixelForValue(riseData.endCurrent);
    
    // Draw arrow
    ctx.save();
    ctx.strokeStyle = '#00ff00';
    ctx.lineWidth = 2;
    ctx.setLineDash([5, 5]);
    
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.stroke();
    
    // Draw label
    ctx.fillStyle = '#00ff00';
    ctx.font = 'bold 12px monospace';
    ctx.fillText(`Rise: ${riseData.riseTime.toFixed(2)}ms`, x1 + 10, y1 - 10);
    
    ctx.restore();
}

function drawPeakLine(ctx, chart, peakCurrent) {
    const xScale = chart.scales.x;
    const yScale = chart.scales.y;
    const chartArea = chart.chartArea;
    
    const y = yScale.getPixelForValue(peakCurrent);
    
    ctx.save();
    ctx.strokeStyle = '#ff6b35';
    ctx.lineWidth = 1;
    ctx.setLineDash([3, 3]);
    
    ctx.beginPath();
    ctx.moveTo(chartArea.left, y);
    ctx.lineTo(chartArea.right, y);
    ctx.stroke();
    
    // Label
    ctx.fillStyle = '#ff6b35';
    ctx.font = 'bold 11px monospace';
    ctx.fillText(`Peak: ${peakCurrent.toFixed(1)}A`, chartArea.right - 100, y - 5);
    
    ctx.restore();
}

function drawWeldStartMarker(ctx, chart) {
    const xScale = chart.scales.x;
    const chartArea = chart.chartArea;
    
    const x = xScale.getPixelForValue(0);
    
    ctx.save();
    ctx.strokeStyle = '#ffff00';
    ctx.lineWidth = 2;
    ctx.setLineDash([]);
    
    ctx.beginPath();
    ctx.moveTo(x, chartArea.top);
    ctx.lineTo(x, chartArea.bottom);
    ctx.stroke();
    
    // Label
    ctx.fillStyle = '#ffff00';
    ctx.font = 'bold 12px monospace';
    ctx.fillText('t=0 (Weld Start)', x + 5, chartArea.top + 20);
    
    ctx.restore();
}

function drawMetricsOverlay(ctx, chartArea, riseData, peakCurrent) {
    if (!riseData) return;
    
    ctx.save();
    ctx.fillStyle = 'rgba(0, 0, 0, 0.7)';
    ctx.fillRect(chartArea.left + 10, chartArea.top + 10, 180, 80);
    
    ctx.fillStyle = '#00ff00';
    ctx.font = 'bold 13px monospace';
    ctx.fillText('📊 Weld Metrics', chartArea.left + 20, chartArea.top + 30);
    
    ctx.font = '12px monospace';
    ctx.fillStyle = '#fff';
    ctx.fillText(`Rise Time: ${riseData.riseTime.toFixed(2)} ms`, chartArea.left + 20, chartArea.top + 50);
    ctx.fillText(`Peak: ${peakCurrent.toFixed(1)} A`, chartArea.left + 20, chartArea.top + 68);
    
    ctx.restore();
}

// Smoothing function
function smoothData(data, windowSize = 3) {
    if (data.length < windowSize) return data;
    
    const smoothed = [];
    for (let i = 0; i < data.length; i++) {
        const start = Math.max(0, i - Math.floor(windowSize / 2));
        const end = Math.min(data.length, i + Math.ceil(windowSize / 2));
        const window = data.slice(start, end);
        const avg = window.reduce((sum, d) => sum + d.y, 0) / window.length;
        smoothed.push({ x: data[i].x, y: avg });
    }
    return smoothed;
}

// Auto-zoom function
function autoZoomChart(chart) {
    const data = chart.data.datasets[0].data;
    if (data.length === 0) return;
    
    const times = data.map(d => d.x);
    const minTime = Math.min(...times);
    const maxTime = Math.max(...times);
    
    // Add 10% padding
    const padding = (maxTime - minTime) * 0.1;
    
    chart.options.scales.x.min = minTime - padding;
    chart.options.scales.x.max = maxTime + padding;
}

