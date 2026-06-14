// import React from 'react';
// import { PanelProps } from '@grafana/data';
// import { SimpleOptions } from 'types';
// import { css, cx } from '@emotion/css';
// import { useStyles2, useTheme2 } from '@grafana/ui';
// import { PanelDataErrorView } from '@grafana/runtime';

// interface Props extends PanelProps<SimpleOptions> {}

// const getStyles = () => {
//   return {
//     wrapper: css`
//       font-family: Open Sans;
//       position: relative;
//     `,
//     svg: css`
//       position: absolute;
//       top: 0;
//       left: 0;
//     `,
//     textBox: css`
//       position: absolute;
//       bottom: 0;
//       left: 0;
//       padding: 10px;
//     `,
//   };
// };

// export const SimplePanel: React.FC<Props> = ({ options, data, width, height, fieldConfig, id }) => {
//   const theme = useTheme2();
//   const styles = useStyles2(getStyles);

//   if (data.series.length === 0) {
//     return <PanelDataErrorView fieldConfig={fieldConfig} panelId={id} data={data} needsStringField />;
//   }

//   return (
//     <div
//       className={cx(
//         styles.wrapper,
//         css`
//           width: ${width}px;
//           height: ${height}px;
//         `
//       )}
//     >
//       <svg
//         className={styles.svg}
//         width={width}
//         height={height}
//         xmlns="http://www.w3.org/2000/svg"
//         xmlnsXlink="http://www.w3.org/1999/xlink"
//         viewBox={`-${width / 2} -${height / 2} ${width} ${height}`}
//       >
//         <g>
//           <circle data-testid="simple-panel-circle" style={{ fill: theme.colors.primary.main }} r={100} />
//         </g>
//       </svg>

//       <div className={styles.textBox}>
//         {options.showSeriesCount && (
//           <div data-testid="simple-panel-series-counter">Number of series: {data.series.length}</div>
//         )}
//         <div>Text option value: {options.text}</div>
//       </div>
//     </div>
//   );
// };

// 1. Add useState and useEffect to imports
import React, { useState, useEffect } from 'react';
import { PanelProps } from '@grafana/data';
import { SimpleOptions } from 'types';
import { css, cx } from '@emotion/css';
import { useStyles2 } from '@grafana/ui';
import { InfluxDBClient } from '@influxdata/influxdb3-client';

// 1. DEFINE THE SHAPE OF YOUR DATA
// This tells TypeScript exactly what columns to expect from InfluxDB.
interface RobotHealth {
  avg_health: number;
  id: number;
  fr_wheel: number; 
  fl_wheel: number;       
  rr_wheel: number;       
  rl_wheel: number;      
  time: string;
}

// 2. STYLES
const getStyles = () => {
  return {
    wrapper: css`
      font-family: Open Sans;
      position: relative;
      width: 100%;
      height: 100%;
      overflow: auto;
    `,
    table: css`
      width: 100%;
      border-collapse: collapse;
      margin-top: 10px;
      th, td {
        border: 1px solid #555;
        padding: 10px;
        text-align: center;
        color: #e0e0e0;
      }
      th {
        background-color: #333;
        font-weight: bold;
        text-transform: uppercase;
      }
      tr:nth-child(even) {
        background-color: #222;
      }
    `
  };
};

// 3. MAIN COMPONENT
export const SimplePanel: React.FC<PanelProps<SimpleOptions>> = ({ options, data, width, height }) => {
  const styles = useStyles2(getStyles);
  
  // CHANGE: specific type <RobotHealth[]> instead of <any[]>
  const [robotData, setRobotData] = useState<RobotHealth[]>([]);

  useEffect(() => {
    const fetchData = async () => {
      try {
        const client = new InfluxDBClient({
          host: 'http://localhost:8086',
          token: 'apiv3_CL-NZqu4HRpOYR7ENJmN3kwgRXCaFpNEEC_j0QHrBqwaMf-XKoFfWWbUq9Oh83xQKEZNGNI9PO44UPPTtr6OKQ',
          database: 'sensors'
        });

        const query = 'SELECT * FROM robot_health ORDER BY time DESC LIMIT 10';
        const stream = client.query(query);
        
        // CHANGE: defined the array type here too
        const results: RobotHealth[] = [];
        
        for await (const row of stream) {
          // We cast 'row' to RobotHealth so TS knows it matches our interface
          // Note: In a real app, you might want to validate data here
          results.push({
            time: String(row.time),
            id: Number(row.id),
            avg_health: Number(row.avg_health),
            fr_wheel: Number(row.fr_wheel),
            fl_wheel: Number(row.fl_wheel),
            rr_wheel: Number(row.rr_wheel),
            rl_wheel: Number(row.rl_wheel),
          });
        }
        setRobotData(results);
      } catch (error) {
        console.error("Failed to fetch data:", error);
      }
    };

    fetchData();

    const timer = setInterval(() => {
      fetchData();
    }, 5000);

    return () => {
      clearInterval(timer);
    };

  }, []); 

  return (
    <div
      className={cx(
        styles.wrapper,
        css`
          width: ${width}px;
          height: ${height}px;
        `
      )}
    >
      <h2>Wheels Stats</h2>
      
      <table className={styles.table}>
        <thead>
          <tr>
            <th>Time</th>
            <th>ID</th>
            <th>Average Health</th>
            <th>Wheel RF</th>
            <th>Wheel LF</th>
            <th>Wheel RR</th>
            <th>Wheel LR</th>
          </tr>
        </thead>
        <tbody>
          {robotData.map((row, index) => (
            <tr key={index}>
              {/* Now TypeScript knows 'row' has these specific properties! */}
              <td>{row.time}</td>
              <td>{row.id}</td>
              <td>{row.avg_health}</td>
              <td>{row.fr_wheel}</td>
              <td>{row.fl_wheel}</td>
              <td>{row.rr_wheel}</td>
              <td>{row.rl_wheel}</td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
};