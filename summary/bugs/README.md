# Bug Reports & Problem Investigations

This folder contains detailed bug reports and problem investigations for the NRP ROS system.

## Purpose

Document issues discovered in the system with:
- Detailed problem description
- Reproduction steps
- Diagnostic data and findings
- Root cause analysis
- Impact assessment

## Current Status

**No standalone bug reports** in this folder.

Bug documentation was integrated directly into fix documentation (see `summary/fixes/`).

## For Future Use

When documenting bugs, include:

### 1. Problem Description
- What is the observed behavior?
- What is the expected behavior?
- How does it impact the system?

### 2. Reproduction Steps
```markdown
1. Start the system with...
2. Navigate to...
3. Observe that...
```

### 3. Diagnostic Data
- Log outputs
- Error messages
- Screenshots/recordings
- Data samples

### 4. Root Cause Analysis
- Where does the bug originate?
- What component is responsible?
- Why does it occur?

### 5. Related Files
- Affected source files
- Diagnostic scripts created
- Test cases

---

## Template

Use this template for new bug reports:

```markdown
# [BUG] Brief Description

**Date Discovered**: YYYY-MM-DD  
**Severity**: Critical / High / Medium / Low  
**Status**: Open / Investigating / Fixed  

## Problem Description

[Detailed description of the issue]

## Observed Behavior

[What actually happens]

## Expected Behavior

[What should happen]

## Reproduction Steps

1. [Step 1]
2. [Step 2]
3. [Step 3]

## Diagnostic Data

### Error Messages
```
[Error output]
```

### System State
- [Relevant system information]

## Root Cause

[Analysis of what causes the issue]

## Impact

[How this affects the system/users]

## Related Files

- [Source file 1]
- [Source file 2]

## Workaround

[Temporary workaround if available]

---

**Reported By**: [Name]  
**Assignee**: [Name]  
**Related Fix**: `summary/fixes/[FIX_FILE.md]`
```

---

**Folder**: `summary/bugs/`  
**Last Updated**: October 30, 2025
