import csv
import numpy as np
import plotly.graph_objects as go


# CONST DEFINITIONS
adc_out_max = 3409
adc_out_min = 227

def read_csv(filename) -> np.array:
    """Read and process a CSV file"""
    try:
        with open(filename, 'r') as file:
            csv_reader = csv.reader(file)
            
            # Get headers
            headers = next(csv_reader)
            print(f"📋 Headers: {headers}\n")
            
            data_arr = []

            row_count = 0
            for row in csv_reader:
                row_count += 1
                
                brake_data = row[3]
                data_arr.append(brake_data)

            print(f"\n✓ Total rows processed: {row_count}")

            return np.array(data_arr, dtype=float)
            
    except FileNotFoundError:
        print(f"Error: '{filename}' not found!")
    except Exception as e:
        print(f"An error occurred: {e}")

def get_brake_percentage_from_analog(input) -> np.array:
    percentages_output = (input - 227) / (adc_out_max - adc_out_min)
    return percentages_output

def plot_brake_bias(front_brake_bias):
    # After calculating brake bias
    fig = go.Figure()

    fig.add_trace(go.Scatter(
        y=front_brake_bias,
        mode='lines',
        name='Front Brake Bias'
    ))

    fig.update_layout(
        title='Brake Bias Over Time',
        xaxis_title='Sample',
        yaxis_title='Brake Bias (%)',
        hovermode='x unified'
    )

    fig.show()

def save_to_csv(data, filename='brake_bias_percentages_output.csv'):
    """Save brake bias data to CSV file"""
    try:
        np.savetxt(filename, data, delimiter=',', header='Front Brake Bias', comments='')
        print(f"✓ CSV saved to {filename}")
    except Exception as e:
        print(f"Error saving CSV: {e}")

def main():
    """Main function"""
    print("=== CSV Reader Script ===\n")
    
    # Change this to your CSV filename
    front_brake_csv_file = 'front_brake_pressure.csv'
    rear_brake_csv_file = 'rear_brake_pressure.csv'

    front_brake_analog = read_csv(front_brake_csv_file)
    rear_brake_analog = read_csv(rear_brake_csv_file)

    front_percentages = get_brake_percentage_from_analog(front_brake_analog)
    rear_percentages = get_brake_percentage_from_analog(rear_brake_analog)
    
    print("FIRST 10 PERCENTAGES:")
    for i in range(10):
        print("FRONT: ", front_percentages[i])
        print("REAR: ", rear_percentages[i])
    
    front_brake_bias = front_percentages / (front_percentages + rear_percentages) * 100

    print("FIRST 10 FRONT BIAS:")
    for i in range(10):
        print(front_brake_bias[i])

    plot_brake_bias(front_brake_bias)

    # At the end, prompt user for output csv
    while True:
        user_input = input("\nGenerate output CSV? (yes/no): ").strip().lower()
        
        if user_input in ['yes', 'y']:
            save_to_csv(front_brake_bias)
            break
        elif user_input in ['no', 'n']:
            print("Skipped CSV generation")
            break
        else:
            print("Please enter 'yes' or 'no'")

    print("\n✓ Done!")

if __name__ == '__main__':
    main()