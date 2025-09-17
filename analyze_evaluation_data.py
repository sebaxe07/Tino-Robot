import pandas as pd
import numpy as np

# Read the CSV file with proper encoding
df = pd.read_csv(r'c:\Users\sebas\Documents\GitHub\Tino-Robot\The Revenge of the Mazes(1-10).csv', encoding='latin1')

print("=== TECHNICAL EVALUATION DATA ANALYSIS ===")
print(f"Total participants: {len(df)}")
print(f"Total columns: {len(df.columns)}")

# Analyze timing data
print("\n=== SESSION TIMING ANALYSIS ===")
if 'Start time' in df.columns and 'Completion time' in df.columns:
    # Convert to datetime and calculate duration
    df['Start time'] = pd.to_datetime(df['Start time'], errors='coerce')
    df['Completion time'] = pd.to_datetime(df['Completion time'], errors='coerce')
    df['Duration'] = (df['Completion time'] - df['Start time']).dt.total_seconds() / 60  # minutes
    
    valid_durations = df['Duration'].dropna()
    if len(valid_durations) > 0:
        print(f"Average session duration: {valid_durations.mean():.1f} minutes")
        print(f"Min session duration: {valid_durations.min():.1f} minutes") 
        print(f"Max session duration: {valid_durations.max():.1f} minutes")
        print(f"Standard deviation: {valid_durations.std():.1f} minutes")

# VR Navigation and Control Technical Aspects
print("\n=== VR NAVIGATION & CONTROL PERFORMANCE ===")
vr_navigation_cols = [
    'I was able to navigate the environment',
    'it was easy to move around', 
    'It was intuitive to move the virtual body',
    'I felt like I could control the virtual body as if it was my own body',
    'the movements of the virtual body were caused by my movements'
]

for col in vr_navigation_cols:
    if col in df.columns:
        print(f"\n{col}:")
        counts = df[col].value_counts()
        total = counts.sum()
        for value, count in counts.items():
            if pd.notna(value):
                percentage = (count/total)*100
                print(f"  {value}: {count} ({percentage:.1f}%)")

# Robot Control and Understanding
print("\n=== ROBOT CONTROL & COMMUNICATION PERFORMANCE ===")
robot_control_cols = [
    'The robot\'s behavior seemed like it was reacting to me.',
    'I found it easy to understand what the robot was doing',
    'It was clear what I needed to do to collaborate with the robot during the task.',
    'The robot\'s indication of which button to press was easy to understand.',
    'The robot\'s actions influenced my decision during the experiment.'
]

for col in robot_control_cols:
    if col in df.columns:
        print(f"\n{col}:")
        counts = df[col].value_counts()
        total = counts.sum()
        for value, count in counts.items():
            if pd.notna(value):
                percentage = (count/total)*100
                print(f"  {value}: {count} ({percentage:.1f}%)")

# Technical Issues and Feedback
print("\n=== TECHNICAL ISSUES & USER FEEDBACK ===")
feedback_cols = [
    'What did you like the least about the experience?',
    'What did you like the least about the experience?2',
    'Any final comments, suggestions, or rants?',
    'Any final comments, suggestions, or rants?2'
]

for col in feedback_cols:
    if col in df.columns:
        print(f"\n{col}:")
        feedback = df[col].dropna()
        for idx, comment in feedback.items():
            if comment and len(str(comment).strip()) > 0:
                print(f"  - {comment}")

# Completion and success rates
print("\n=== COMPLETION RATES ===")
completion_cols = [
    'Did you finish the game in this room?',
    'Did you cross the line to enter into the square?'
]

for col in completion_cols:
    if col in df.columns:
        print(f"\n{col}:")
        counts = df[col].value_counts()
        total = counts.sum()
        for value, count in counts.items():
            if pd.notna(value):
                percentage = (count/total)*100
                print(f"  {value}: {count} ({percentage:.1f}%)")

# Experience type analysis
print("\n=== EXPERIENCE TYPE ANALYSIS ===")
if 'Did you take part in the VR Experience?' in df.columns:
    vr_participants = df['Did you take part in the VR Experience?'].value_counts()
    print("VR Experience participation:")
    for value, count in vr_participants.items():
        if pd.notna(value):
            print(f"  {value}: {count}")

if 'Did you take part in the Robot experience?' in df.columns:
    robot_participants = df['Did you take part in the Robot experience?'].value_counts()
    print("\nRobot Experience participation:")
    for value, count in robot_participants.items():
        if pd.notna(value):
            print(f"  {value}: {count}")