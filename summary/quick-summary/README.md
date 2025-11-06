# Quick Summary & Status Pages

This folder contains concise, single-page references for quick access during development and troubleshooting.

## Files

### GPS_ISSUE_QUICK_REF.md ⭐
**Purpose**: One-page GPS issue reference card

**Contains**:
- Problem summary
- Solution overview
- Quick test command
- Current status

**When to Use**: Need immediate answer about GPS fix without reading detailed docs

**Quick View**:
```
Problem: Altitude -76.5m → Solution: Raw GPS topic → Result: +16.6m ✅
```

---

### CURRENT_SYSTEM_STATUS.md 📊
**Purpose**: Real-time system status snapshot

**Contains**:
- Working components
- Current telemetry values
- Recent changes
- Quick commands

**When to Use**: 
- System health check
- Before starting work
- After system restart
- When onboarding new developers

**Quick Check**:
```bash
# View system status
cat summary/quick-summary/CURRENT_SYSTEM_STATUS.md
```

---

## Purpose of Quick Summaries

Quick summaries are designed for:

✅ **Rapid Reference**
- Get answer in < 30 seconds
- No need to read full documentation
- Essential info only

✅ **Status Checks**
- Verify system health
- Check recent changes
- Confirm current configuration

✅ **Troubleshooting Entry Point**
- Start here when investigating issues
- Links to detailed documentation
- Common commands readily available

✅ **Team Communication**
- Share current status quickly
- Onboard new team members
- Document system state

---

## Quick Summary Standards

Each quick summary should:

1. **Fit on One Screen**
   - Maximum 50 lines
   - No scrolling needed
   - Scannable format

2. **Focus on Essential Info**
   - Problem/Solution/Result
   - Current status
   - Key metrics
   - Critical commands

3. **Link to Details**
   - Reference detailed docs
   - Point to test scripts
   - Link to related files

4. **Stay Current**
   - Update after major changes
   - Include date
   - Reflect production state

---

## Template for New Quick Summaries

```markdown
# [Topic] - Quick Reference

**Last Updated**: YYYY-MM-DD  
**Status**: ✅ / ⚠️ / ❌

## 🎯 Quick Answer

[One-sentence answer to most common question]

## 📊 Current State

| Item | Value | Status |
|------|-------|--------|
| Metric1 | Value1 | ✅ |
| Metric2 | Value2 | ✅ |

## 🔧 Quick Commands

```bash
# Do thing 1
command1

# Do thing 2
command2
```

## 📚 More Info

- Detailed docs: `path/to/detailed.md`
- Tests: `path/to/test.py`
- Related: `path/to/related.md`

---
**Page**: `summary/quick-summary/[FILE].md`
```

---

## Creating New Quick Summaries

Good candidates for quick summaries:

1. **Frequently Asked Questions**
   - "What's the current GPS status?"
   - "How do I restart services?"
   - "What tests should I run?"

2. **System States**
   - Current configuration
   - Active features
   - Known issues

3. **Common Procedures**
   - Deployment steps
   - Debugging workflow
   - Test execution

---

## Update Schedule

Update quick summaries when:
- ✅ Major feature added
- ✅ Critical fix deployed
- ✅ System configuration changed
- ✅ New component added
- ✅ Status changes significantly

Quick summaries should always reflect the **current production state**.

---

## Related Documentation

- **Detailed Docs**: `summary/fixes/`, `summary/bugs/`
- **Technical Summaries**: `summary/brief-summary/`
- **Daily Reports**: `summary/daily-reports/`
- **Tests**: `summary/tests/`

---

**Folder**: `summary/quick-summary/`  
**Last Updated**: October 30, 2025  
**Current Files**: 2  
**Purpose**: Rapid reference and status checks
